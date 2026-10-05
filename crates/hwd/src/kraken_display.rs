//! Service-owned Kraken 2023 Standard LCD controller.
use std::{
    cell::RefCell,
    fs,
    path::Path,
    sync::{Arc, Condvar, Mutex, OnceLock},
    thread::{self, JoinHandle},
    time::Duration,
};

use nzxt_cam_core::{Device, DeviceId, KrakenDisplayMode, KrakenDisplaySnapshot, ReadingKind};
use swash::{
    FontRef,
    scale::{Render, ScaleContext, Source},
    zeno::Format,
};

use crate::{
    config::HardwareConfig,
    hardware::{HardwareError, HardwareErrorKind, HardwareOperations},
    liquidctl::LiquidctlHardware,
    telemetry::HostTelemetry,
};

const TICK: Duration = Duration::from_secs(2);

#[derive(Default)]
struct State {
    snapshot: KrakenDisplaySnapshot,
    revision: u64,
    applied_revision: Option<u64>,
    stopped: bool,
}

type Shared = Arc<(Mutex<State>, Condvar)>;

/// Owns its own USB adapter, host sensors and serial upload loop, not the client connection.
pub struct KrakenDisplayWorker {
    shared: Shared,
    thread: Option<JoinHandle<()>>,
}

impl KrakenDisplayWorker {
    pub fn new(config: HardwareConfig, io_lock: Arc<Mutex<()>>) -> Self {
        Self::with_sources(
            LiquidctlHardware::with_io_lock(io_lock),
            HostTelemetry::new(config),
        )
    }

    pub(crate) fn with_sources(hardware: LiquidctlHardware, host: HostTelemetry) -> Self {
        let shared: Shared = Arc::new((Mutex::new(State::default()), Condvar::new()));
        let thread_shared = shared.clone();
        let thread = thread::Builder::new()
            .name("kraken-display".into())
            .spawn(move || run(thread_shared, hardware, host))
            .expect("could not start Kraken display worker");
        Self {
            shared,
            thread: Some(thread),
        }
    }

    pub(crate) fn applied_selection(&self, id: &DeviceId, mode: KrakenDisplayMode) -> bool {
        let state = self.shared.0.lock().unwrap();
        state.applied_revision == Some(state.revision)
            && state.snapshot.device_id.as_ref() == Some(id)
            && state.snapshot.mode == mode
    }

    pub fn snapshot(&self) -> KrakenDisplaySnapshot {
        self.shared.0.lock().unwrap().snapshot.clone()
    }

    /// Forget an old selection without issuing a display reset command.
    pub(crate) fn clear_selection(&self) {
        let mut state = self.shared.0.lock().unwrap();
        state.snapshot = KrakenDisplaySnapshot::default();
        state.revision = state.revision.wrapping_add(1);
        state.applied_revision = None;
        self.shared.1.notify_one();
    }

    /// Interactive selection is accepted after fresh USB identity validation by the manager.
    pub fn select(&self, device_id: DeviceId, mode: KrakenDisplayMode) {
        self.select_with_status(device_id, mode, None);
    }

    /// A restored selection is intent, not evidence that an LCD command has
    /// completed. The pending marker clears only after a successful upload.
    pub fn select_restored(&self, device_id: DeviceId, mode: KrakenDisplayMode) {
        self.select_with_status(device_id, mode, Some("LCD restore pending upload".into()));
    }

    fn select_with_status(
        &self,
        device_id: DeviceId,
        mode: KrakenDisplayMode,
        status: Option<String>,
    ) {
        let mut state = self.shared.0.lock().unwrap();
        state.snapshot = KrakenDisplaySnapshot {
            device_id: Some(device_id),
            mode,
            last_error: status,
        };
        state.revision = state.revision.wrapping_add(1);
        state.applied_revision = None;
        self.shared.1.notify_one();
    }
}

impl Drop for KrakenDisplayWorker {
    fn drop(&mut self) {
        {
            let mut state = self.shared.0.lock().unwrap();
            state.stopped = true;
            self.shared.1.notify_one();
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn run(shared: Shared, mut hardware: LiquidctlHardware, mut host: HostTelemetry) {
    // The directory is mode 0700, the file is exclusively service-created, and
    // both are removed automatically on exit. No client-supplied path is used.
    let temp = tempfile::Builder::new()
        .prefix("nzxt-kraken-display-")
        .tempdir();
    let mut processed = None;
    loop {
        let (selection, revision) = {
            let (lock, changed) = &*shared;
            let mut state = lock.lock().unwrap();
            while !state.stopped
                && (state.snapshot.device_id.is_none() || processed == Some(state.revision))
            {
                state = changed.wait(state).unwrap();
            }
            if state.stopped {
                break;
            }
            (state.snapshot.clone(), state.revision)
        };
        let id = selection.device_id.as_ref().unwrap();
        let result = match &temp {
            Ok(dir) => tick(
                &mut hardware,
                &mut host,
                id,
                selection.mode,
                &dir.path().join("display.png"),
            ),
            Err(error) => Err(HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                format!("cannot create private LCD image directory: {error}"),
            )),
        };
        let successful = result.is_ok();
        let mut state = shared.0.lock().unwrap();
        if revision == state.revision {
            state.applied_revision = successful.then_some(revision);
            state.snapshot.last_error = result.err().map(|error| error.message().to_owned());
        }
        processed = Some(revision);
        if state.stopped {
            break;
        }
        if state.revision != revision {
            continue;
        }
        // Firmware's built-in liquid display updates itself: issue its command
        // once per selection; only failed commands are retried.
        if selection.mode == KrakenDisplayMode::BuiltinLiquid && successful {
            continue;
        }
        // Refresh the selected LCD image (not a firmware fan curve) after the
        // previous bounded upload. Retry failures against the same DeviceId.
        let (state, wait) = shared
            .1
            .wait_timeout_while(state, TICK, |state| {
                !state.stopped && state.revision == revision
            })
            .unwrap();
        if state.stopped {
            break;
        }
        if wait.timed_out() {
            processed = None;
        }
    }
}

fn tick(
    hardware: &mut LiquidctlHardware,
    host: &mut HostTelemetry,
    id: &DeviceId,
    mode: KrakenDisplayMode,
    image: &Path,
) -> Result<(), HardwareError> {
    let serial = hardware.display_serial(id)?;
    if mode == KrakenDisplayMode::BuiltinLiquid {
        return hardware.set_display(&serial, None);
    }
    // Sample BOTH sources on every iteration. Never cache temperatures across
    // errors, offline transitions, or mode changes.
    let usb = match hardware.snapshot() {
        Ok(snapshot) => snapshot
            .devices
            .into_iter()
            .find(|device| &device.id == id && device.online),
        Err(_) => None,
    };
    let host = host.sample_device();
    let values = [
        temperature(host.as_ref(), "CPU Tctl"),
        temperature(host.as_ref(), "NVIDIA GPU"),
        temperature(usb.as_ref(), "Liquid temperature"),
    ];
    let png = render(mode, values);
    fs::write(image, png).map_err(|error| {
        HardwareError::with_kind(
            HardwareErrorKind::Unavailable,
            format!("cannot write private LCD image: {error}"),
        )
    })?;
    hardware.set_display(&serial, Some(image))
}

fn temperature(device: Option<&Device>, label: &str) -> Option<f64> {
    device
        .filter(|device| device.online)?
        .readings
        .iter()
        .find(|reading| reading.label == label && reading.kind == ReadingKind::Temperature)
        .map(|reading| reading.value)
        .filter(|value| value.is_finite() && (-20.0..=150.0).contains(value))
}

fn fields(mode: KrakenDisplayMode) -> &'static [usize] {
    match mode {
        KrakenDisplayMode::BuiltinLiquid => {
            unreachable!("built-in liquid mode is handled by device firmware")
        }
        KrakenDisplayMode::Cpu => &[0],
        KrakenDisplayMode::Gpu => &[1],
        KrakenDisplayMode::Liquid => &[2],
        KrakenDisplayMode::CpuGpu => &[0, 1],
        KrakenDisplayMode::CpuLiquid => &[0, 2],
        KrakenDisplayMode::GpuLiquid => &[1, 2],
        KrakenDisplayMode::CpuGpuLiquid => &[0, 1, 2],
    }
}

const LCD_SIZE: usize = 240;
const FONT_BYTES: &[u8] = include_bytes!("../assets/NotoSans-Bold.ttf");
type Font = FontRef<'static>;
static FONT: OnceLock<Font> = OnceLock::new();

thread_local! {
    static SCALE_CONTEXT: RefCell<ScaleContext> = RefCell::new(ScaleContext::new());
}

fn display_font() -> &'static Font {
    FONT.get_or_init(|| {
        FontRef::from_index(FONT_BYTES, 0)
            .expect("bundled Noto Sans Bold must be a valid TrueType font")
    })
}

fn display_value(value: Option<f64>) -> String {
    value.map_or_else(
        || "—".to_owned(),
        |value| format!("{}", value.round() as i32),
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TextBox {
    x: i32,
    y: i32,
    width: i32,
    height: i32,
}

struct RasterGlyph {
    x: i32,
    y: i32,
    width: usize,
    height: usize,
    bitmap: Vec<u8>,
}

struct RasterRun {
    glyphs: Vec<RasterGlyph>,
    bounds: TextBox,
}

fn rasterize(font: &Font, text: &str, size: u32) -> RasterRun {
    SCALE_CONTEXT.with_borrow_mut(|context| {
        let mut scaler = context.builder(*font).size(size as f32).hint(false).build();
        let charmap = font.charmap();
        let metrics = font.glyph_metrics(&[]).scale(size as f32);
        let mut pen = 0.0_f32;
        let mut glyphs = Vec::new();
        let (mut left, mut top, mut right, mut bottom) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
        for character in text.chars() {
            let id = charmap.map(character);
            if let Some(image) = Render::new(&[Source::Outline])
                .format(Format::Alpha)
                .render(&mut scaler, id)
            {
                debug_assert_eq!(image.content, swash::scale::image::Content::Mask);
                let placement = image.placement;
                let width = placement.width as usize;
                let height = placement.height as usize;
                debug_assert_eq!(image.data.len(), width * height);
                // Placement is relative to an upward-pointing baseline; the
                // grayscale mask rows already run from top to bottom.
                let x = pen.round() as i32 + placement.left;
                let y = -placement.top;
                if width > 0 && height > 0 {
                    left = left.min(x);
                    top = top.min(y);
                    right = right.max(x + placement.width as i32);
                    bottom = bottom.max(y + placement.height as i32);
                    glyphs.push(RasterGlyph {
                        x,
                        y,
                        width,
                        height,
                        bitmap: image.data,
                    });
                }
            }
            pen += metrics.advance_width(id);
        }
        let bounds = if glyphs.is_empty() {
            TextBox {
                x: 0,
                y: 0,
                width: 0,
                height: 0,
            }
        } else {
            TextBox {
                x: left,
                y: top,
                width: right - left,
                height: bottom - top,
            }
        };
        RasterRun { glyphs, bounds }
    })
}

/// Fit visible glyph pixels, not font line height, inside a full-bleed panel.
fn draw_fitted_text(
    rgb: &mut [u8],
    font: &Font,
    text: &str,
    box_: TextBox,
    max_size: u32,
    color: [u8; 3],
) -> TextBox {
    let mut low = 1;
    let mut high = max_size;
    while low < high {
        let mid = (low + high).div_ceil(2);
        let bounds = rasterize(font, text, mid).bounds;
        if bounds.width <= box_.width && bounds.height <= box_.height {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    let run = rasterize(font, text, low);
    let origin_x = box_.x - run.bounds.x;
    let origin_y = box_.y + (box_.height - run.bounds.height) / 2 - run.bounds.y;
    for glyph in &run.glyphs {
        for gy in 0..glyph.height {
            for gx in 0..glyph.width {
                let x = origin_x + glyph.x + gx as i32;
                let y = origin_y + glyph.y + gy as i32;
                if !(0..LCD_SIZE as i32).contains(&x) || !(0..LCD_SIZE as i32).contains(&y) {
                    continue;
                }
                let alpha = glyph.bitmap[gy * glyph.width + gx] as u16;
                let offset = (y as usize * LCD_SIZE + x as usize) * 3;
                for component in 0..3 {
                    let background = rgb[offset + component] as u16;
                    rgb[offset + component] = ((background * (255 - alpha)
                        + color[component] as u16 * alpha
                        + 127)
                        / 255) as u8;
                }
            }
        }
    }
    TextBox {
        x: box_.x,
        y: origin_y + run.bounds.y,
        width: run.bounds.width,
        height: run.bounds.height,
    }
}

// Fit the number and its half-size degree sign together so neither can clip
// at the right edge. Keep the degree sign aligned with the number's ink top.
fn draw_temperature(
    rgb: &mut [u8],
    font: &Font,
    value: Option<f64>,
    box_: TextBox,
    max_size: u32,
    color: [u8; 3],
) -> (TextBox, Option<TextBox>) {
    let number = display_value(value);
    if value.is_none() {
        return (
            draw_fitted_text(rgb, font, &number, box_, max_size, color),
            None,
        );
    }
    let gap = 3;
    let mut low = 1;
    let mut high = max_size;
    while low < high {
        let mid = (low + high).div_ceil(2);
        let number_bounds = rasterize(font, &number, mid).bounds;
        let degree_bounds = rasterize(font, "°", (mid / 2).max(1)).bounds;
        if number_bounds.width + gap + degree_bounds.width <= box_.width
            && number_bounds.height <= box_.height
        {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    let number_bounds = rasterize(font, &number, low).bounds;
    let degree_size = (low / 2).max(1);
    let degree_bounds = rasterize(font, "°", degree_size).bounds;
    let number_ink = draw_fitted_text(
        rgb,
        font,
        &number,
        TextBox {
            width: number_bounds.width,
            ..box_
        },
        low,
        color,
    );
    let degree_ink = draw_fitted_text(
        rgb,
        font,
        "°",
        TextBox {
            x: number_ink.x + number_ink.width + gap,
            y: number_ink.y,
            width: degree_bounds.width,
            height: degree_bounds.height,
        },
        degree_size,
        color,
    );
    (number_ink, Some(degree_ink))
}

fn render(mode: KrakenDisplayMode, values: [Option<f64>; 3]) -> Vec<u8> {
    let mut rgb = vec![0u8; LCD_SIZE * LCD_SIZE * 3];
    let font = display_font();
    let slots = fields(mode);
    for (row, &field) in slots.iter().enumerate() {
        let height = LCD_SIZE / slots.len();
        let y = row * height;
        // Every pixel is part of the face: no bezel, ring, inset card or
        // reserved outer margin. Only the glyphs need a small clipping inset.
        for py in y..y + height {
            for px in 0..LCD_SIZE {
                let offset = (py * LCD_SIZE + px) * 3;
                let shade = 20 + (row % 2) as u8 * 4 + (px / 80) as u8;
                rgb[offset..offset + 3].copy_from_slice(&[shade, shade + 2, shade + 6]);
            }
        }
        if row != 0 {
            let offset = y * LCD_SIZE * 3;
            for pixel in rgb[offset..offset + LCD_SIZE * 3].as_chunks_mut::<3>().0 {
                pixel.copy_from_slice(&[49, 53, 61]);
            }
        }

        // Each row uses its entire 240px width. Labels sit immediately above
        // large integer temperatures with a smaller degree sign at the top right.
        let (label_y, label_height, label_size, value_y, value_height, value_size) =
            match slots.len() {
                1 => (41, 54, 57, 97, 111, 148),
                2 => (6, 26, 32, 32, 78, 110),
                _ => (3, 20, 23, 24, 52, 72),
            };
        let y = y as i32;
        draw_fitted_text(
            &mut rgb,
            font,
            ["CPU", "GPU", "LIQUID"][field],
            TextBox {
                x: 7,
                y: y + label_y,
                width: 232,
                height: label_height,
            },
            label_size,
            [[81, 207, 229], [179, 135, 248], [251, 196, 118]][field],
        );
        draw_temperature(
            &mut rgb,
            font,
            values[field],
            TextBox {
                x: 7,
                y: y + value_y,
                width: 232,
                height: value_height,
            },
            value_size,
            if values[field].is_some() {
                [248, 249, 252]
            } else {
                [252, 181, 102]
            },
        );
    }
    png_rgb(LCD_SIZE as u32, LCD_SIZE as u32, &rgb)
}

fn png_rgb(width: u32, height: u32, rgb: &[u8]) -> Vec<u8> {
    let mut raw = Vec::with_capacity(rgb.len() + height as usize);
    for row in rgb.chunks_exact(width as usize * 3) {
        raw.push(0);
        raw.extend_from_slice(row);
    }
    let mut zlib = vec![0x78, 0x01];
    for (i, chunk) in raw.chunks(65535).enumerate() {
        zlib.push(u8::from((i + 1) * 65535 >= raw.len()));
        let len = chunk.len() as u16;
        zlib.extend_from_slice(&len.to_le_bytes());
        zlib.extend_from_slice(&(!len).to_le_bytes());
        zlib.extend_from_slice(chunk);
    }
    let (mut a, mut b) = (1u32, 0u32);
    for &byte in &raw {
        a = (a + byte as u32) % 65521;
        b = (b + a) % 65521;
    }
    zlib.extend_from_slice(&((b << 16) | a).to_be_bytes());
    let mut png = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut ihdr = Vec::from(width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(&mut png, b"IHDR", &ihdr);
    chunk(&mut png, b"IDAT", &zlib);
    chunk(&mut png, b"IEND", &[]);
    png
}

fn chunk(png: &mut Vec<u8>, name: &[u8; 4], data: &[u8]) {
    png.extend_from_slice(&(data.len() as u32).to_be_bytes());
    png.extend_from_slice(name);
    png.extend_from_slice(data);
    let mut crc = !0u32;
    for &byte in name.iter().chain(data) {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    png.extend_from_slice(&(!crc).to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn applied_selection_requires_success_for_current_revision_and_exact_identity() {
        let root = tempfile::tempdir().unwrap();
        let mut worker = KrakenDisplayWorker::with_sources(
            LiquidctlHardware::unavailable_for_test(),
            HostTelemetry::at_empty_test_root(root.path()),
        );
        // No upload thread may race the synthetic completion below.
        {
            let mut state = worker.shared.0.lock().unwrap();
            state.stopped = true;
            worker.shared.1.notify_one();
        }
        worker.thread.take().unwrap().join().unwrap();
        let id = DeviceId::new("lcd");
        worker.select(id.clone(), KrakenDisplayMode::Cpu);
        assert!(!worker.applied_selection(&id, KrakenDisplayMode::Cpu));
        {
            // Simulate a completed successful upload without running a USB command.
            let mut state = worker.shared.0.lock().unwrap();
            state.applied_revision = Some(state.revision);
        }
        assert!(worker.applied_selection(&id, KrakenDisplayMode::Cpu));
        assert!(!worker.applied_selection(&DeviceId::new("other"), KrakenDisplayMode::Cpu));
        worker.select(id.clone(), KrakenDisplayMode::Gpu);
        assert!(!worker.applied_selection(&id, KrakenDisplayMode::Cpu));
        assert!(!worker.applied_selection(&id, KrakenDisplayMode::Gpu));
    }

    #[test]
    fn worker_selection_transitions_and_reports_errors_without_client_or_device_io() {
        let root = tempfile::tempdir().unwrap();
        let mut worker = KrakenDisplayWorker::with_sources(
            LiquidctlHardware::unavailable_for_test(),
            HostTelemetry::at_empty_test_root(root.path()),
        );
        assert_eq!(worker.snapshot(), KrakenDisplaySnapshot::default());
        let id = DeviceId::new("kraken-standard");
        worker.select(id.clone(), KrakenDisplayMode::Cpu);
        assert_eq!(worker.snapshot().device_id, Some(id.clone()));
        let started = std::time::Instant::now();
        while worker.snapshot().last_error.is_none() && started.elapsed() < Duration::from_secs(1) {
            thread::sleep(Duration::from_millis(2));
        }
        assert!(
            worker
                .snapshot()
                .last_error
                .as_deref()
                .unwrap()
                .contains("liquidctl")
        );
        // Check selection's immediate error reset without a new worker result
        // racing the snapshot. The fake upload above still exercises reporting.
        {
            let mut state = worker.shared.0.lock().unwrap();
            state.stopped = true;
            worker.shared.1.notify_one();
        }
        worker.thread.take().unwrap().join().unwrap();
        worker.select(id, KrakenDisplayMode::GpuLiquid);
        let selected = worker.snapshot();
        assert_eq!(selected.mode, KrakenDisplayMode::GpuLiquid);
        assert_eq!(selected.last_error, None);
    }

    #[test]
    fn presets_render_exact_240_square_and_missing_sensor_is_not_reused() {
        assert_ne!(display_font().charmap().map('°'), 0);
        assert_ne!(display_font().charmap().map('—'), 0);
        assert_eq!(display_value(Some(67.24)), "67");
        assert_eq!(display_value(Some(67.5)), "68");
        assert_eq!(display_value(Some(-0.4)), "0");
        assert_eq!(display_value(None), "—");
        for mode in KrakenDisplayMode::ALL.into_iter().skip(1) {
            let png = render(mode, [None, Some(67.2), None]);
            assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
            assert_eq!(&png[16..24], &[0, 0, 0, 240, 0, 0, 0, 240]);
            let mut changed = [None, Some(67.2), None];
            changed[fields(mode)[0]] = Some(45.0);
            assert_ne!(png, render(mode, changed));
        }
        assert_eq!(fields(KrakenDisplayMode::CpuGpuLiquid), &[0, 1, 2]);
    }

    #[test]
    #[should_panic(expected = "built-in liquid mode is handled by device firmware")]
    fn builtin_liquid_is_not_a_host_rendered_preset() {
        render(KrakenDisplayMode::BuiltinLiquid, [None; 3]);
    }

    #[test]
    fn bundled_font_repertoire_has_consistent_grayscale_masks() {
        let font = display_font();
        for character in "-0123456789°—CPUGLIQD".chars() {
            assert_ne!(font.charmap().map(character), 0, "missing {character:?}");
            for size in [23, 32, 57, 72, 110, 148] {
                let run = rasterize(font, &character.to_string(), size);
                assert!(!run.glyphs.is_empty(), "empty {character:?} at {size}");
                for glyph in run.glyphs {
                    assert_eq!(glyph.bitmap.len(), glyph.width * glyph.height);
                    assert!(glyph.bitmap.iter().any(|&alpha| alpha != 0));
                    assert!(glyph.bitmap.iter().any(|&alpha| alpha != 255));
                }
            }
        }
    }

    #[test]
    fn raster_masks_preserve_baseline_and_empty_run_geometry() {
        let font = display_font();
        let descender = rasterize(font, "g", 64);
        assert_eq!(descender.glyphs.len(), 1);
        let glyph = &descender.glyphs[0];
        assert!(glyph.y < 0);
        assert!(glyph.y + glyph.height as i32 > 0);
        for text in ["", "   "] {
            let run = rasterize(font, text, 64);
            assert!(run.glyphs.is_empty());
            assert_eq!(
                run.bounds,
                TextBox {
                    x: 0,
                    y: 0,
                    width: 0,
                    height: 0
                }
            );
        }
    }

    #[test]
    fn parallel_scalers_render_every_preset_deterministically() {
        let expected: Vec<_> = KrakenDisplayMode::ALL
            .into_iter()
            .skip(1)
            .map(|mode| render(mode, [Some(53.2), Some(67.8), None]))
            .collect();
        let workers: Vec<_> = (0..8)
            .map(|_| {
                thread::spawn(|| {
                    KrakenDisplayMode::ALL
                        .into_iter()
                        .skip(1)
                        .map(|mode| render(mode, [Some(53.2), Some(67.8), None]))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        for worker in workers {
            assert_eq!(worker.join().unwrap(), expected);
        }
    }

    #[test]
    #[ignore = "writes example PNGs for visual review; run with --ignored --nocapture"]
    fn write_display_template_previews() {
        let directory =
            std::env::temp_dir().join(format!("nzxt-display-preview-{}", std::process::id()));
        fs::create_dir_all(&directory).unwrap();
        for mode in KrakenDisplayMode::ALL.into_iter().skip(1) {
            let png = render(mode, [Some(53.2), Some(67.8), Some(32.4)]);
            fs::write(directory.join(format!("{mode:?}.png")), png).unwrap();
        }
        fs::write(
            directory.join("MissingGpu.png"),
            render(
                KrakenDisplayMode::CpuGpuLiquid,
                [Some(53.2), None, Some(32.4)],
            ),
        )
        .unwrap();
        println!("Template previews: {}", directory.display());
    }

    #[test]
    fn noto_sans_fits_large_numbers_and_labels_in_every_panel() {
        let font = display_font();
        let mut rgb = vec![0; LCD_SIZE * LCD_SIZE * 3];
        let mut single_value_height = 0;
        let mut double_value_height = 0;
        for (rows, label_y, label_height, label_size, value_y, value_height, value_size) in [
            (1, 41, 54, 57, 97, 111, 148),
            (2, 6, 26, 32, 32, 78, 110),
            (3, 3, 20, 23, 24, 52, 72),
        ] {
            let height = LCD_SIZE as i32 / rows;
            for y in (0..rows).map(|row| row * height) {
                let label_box = TextBox {
                    x: 7,
                    y: y + label_y,
                    width: 232,
                    height: label_height,
                };
                let value_box = TextBox {
                    x: 7,
                    y: y + value_y,
                    width: 232,
                    height: value_height,
                };
                assert!(label_box.y + label_box.height <= value_box.y);
                assert!(value_box.y + value_box.height <= y + height);
                for label in ["CPU", "GPU", "LIQUID"] {
                    let ink =
                        draw_fitted_text(&mut rgb, font, label, label_box, label_size, [255; 3]);
                    assert!(ink.width > 0 && ink.x + ink.width <= LCD_SIZE as i32);
                    assert!(ink.y >= label_box.y && ink.y + ink.height <= value_box.y);
                }
                for value in [Some(-20.0), Some(150.0), Some(32.0), None] {
                    let (number, degree) =
                        draw_temperature(&mut rgb, font, value, value_box, value_size, [255; 3]);
                    assert!(number.width > 0 && number.height > 0, "{value:?}");
                    assert!(
                        number.x >= value_box.x
                            && number.x + number.width <= value_box.x + value_box.width
                    );
                    assert!(
                        number.y >= value_box.y
                            && number.y + number.height <= value_box.y + value_box.height
                    );
                    if let Some(degree) = degree {
                        assert_eq!(degree.x, number.x + number.width + 3);
                        assert_eq!(degree.y, number.y);
                        assert!(degree.height < number.height);
                        assert!(degree.x + degree.width <= value_box.x + value_box.width);
                        assert!(degree.y + degree.height <= value_box.y + value_box.height);
                    } else {
                        assert!(
                            value.is_none(),
                            "only a missing reading omits the degree sign"
                        );
                    }
                    if value == Some(32.0) && y == 0 {
                        if rows == 1 {
                            single_value_height = number.height;
                        }
                        if rows == 2 {
                            double_value_height = number.height;
                        }
                    }
                }
            }
        }
        assert!(single_value_height > double_value_height);
    }
}
