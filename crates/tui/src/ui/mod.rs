mod curve_chart;

use ratatui::{
    Frame,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    symbols::border,
    text::{Line, Span},
    widgets::{Block, BorderType, Borders, Clear, Padding, Paragraph, Wrap},
};

use crate::{
    app::{App, EditorMode, Modal, StatusKind},
    model::{
        Device, DeviceId, DeviceKind, HostControlState, KrakenDisplayMode, MonitoringActualState,
        MonitoringTargetIntent, ReadingKind,
    },
    profile::ProfileKind,
    theme::Theme,
};

use self::curve_chart::CurveChart;

pub const MIN_WIDTH: u16 = 88;
pub const MIN_HEIGHT: u16 = 30;

pub fn draw(frame: &mut Frame, app: &App) {
    let theme = Theme::monochrome();
    let area = frame.area();

    if area.width < MIN_WIDTH || area.height < MIN_HEIGHT {
        render_too_small(frame, area, theme);
        return;
    }

    let has_host_error = app.snapshot.host_control.last_error.is_some();
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Length(12),
            Constraint::Min(if has_host_error { 12 } else { 13 }),
            Constraint::Length(if has_host_error { 3 } else { 2 }),
        ])
        .split(area);

    render_header(frame, rows[0], app, theme);
    render_device_overview(frame, rows[1], app, theme);
    if app.needs_opt_in() {
        render_opt_in(frame, rows[2], app, theme);
    } else {
        render_curve_editor(frame, rows[2], app, theme);
    }
    render_footer(frame, rows[3], app, theme);

    match app.modal {
        Some(Modal::Help) => render_help(frame, area, app, theme),
        Some(Modal::Profiles) => render_profile_picker(frame, area, app, theme),
        Some(Modal::HostProfiles) => render_host_profile_picker(frame, area, app, theme),
        Some(Modal::HostApplyScope) => render_host_apply_scope(frame, area, app, theme),
        Some(Modal::RenameProfile) => render_profile_name_input(frame, area, app, theme),
        Some(Modal::ConfirmApply) => render_apply_confirmation(frame, area, app, theme),
        Some(Modal::KrakenDisplay) => render_display_picker(frame, area, app, theme),
        Some(Modal::Settings) => render_settings(frame, area, app, theme),
        None => {}
    }
}

fn render_header(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let block = Block::default()
        .borders(Borders::BOTTOM)
        .border_set(border::THICK)
        .border_style(theme.border);
    frame.render_widget(block, area);

    let left = Line::from(vec![
        Span::styled(" NZXT//CAM TUI ", theme.selected),
        Span::styled("  HARDWARE CONTROL CONSOLE", theme.muted),
    ]);
    let demo = app.backend_name == "DEMO";
    let left_width = area.width.saturating_sub(if demo { 6 } else { 0 });
    frame.render_widget(
        Paragraph::new(left).style(theme.text),
        Rect::new(area.x, area.y, left_width, 1),
    );
    if demo {
        frame.render_widget(
            Paragraph::new(Line::styled(" DEMO ", theme.strong)),
            Rect::new(area.right() - 6, area.y, 6, 1),
        );
    }
}

fn render_device_overview(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let columns = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(36),
            Constraint::Percentage(34),
            Constraint::Percentage(30),
        ])
        .spacing(1)
        .split(area);

    let active_id = app
        .active_device()
        .filter(|_| app.editor_mode == EditorMode::Firmware)
        .map(|device| &device.id);
    for ((kind, column), index) in DeviceKind::ALL.iter().zip(columns.iter()).zip(0_usize..) {
        render_device_group(frame, *column, app, *kind, active_id, theme, index);
    }
}

fn render_device_group(
    frame: &mut Frame,
    area: Rect,
    app: &App,
    kind: DeviceKind,
    active_id: Option<&DeviceId>,
    theme: Theme,
    index: usize,
) {
    let devices = app
        .snapshot
        .devices
        .iter()
        .filter(|device| device.kind == kind)
        .collect::<Vec<_>>();
    let group_is_active = (app.editor_mode == EditorMode::Host
        && kind == DeviceKind::FanController)
        || active_id.is_some_and(|active| devices.iter().any(|device| &device.id == active));
    let count = devices.len();
    let title = Line::from(vec![
        Span::styled(format!(" 0{} ", index + 1), theme.selected),
        Span::styled(format!(" {} ", kind.title()), theme.strong),
        Span::styled(format!(" {count:02} "), theme.muted),
    ]);
    let mut block = Block::bordered()
        .border_type(BorderType::Plain)
        .border_style(if group_is_active {
            theme.focused_border
        } else {
            theme.border
        })
        .title(title)
        .padding(Padding::horizontal(1));
    if kind == DeviceKind::FanController {
        // Use the border, not a telemetry row: all existing RPM readings must
        // remain visible in this fixed-height overview.
        block = block.title_bottom(Line::styled(
            format!(" Board: {} ", host_state_label(app)),
            theme.muted,
        ));
    } else if kind == DeviceKind::LiquidCooler && app.snapshot.kraken_display.device_id.is_some() {
        block = block.title_bottom(Line::styled(
            if app.host_status_trusted {
                let state =
                    app.snapshot.monitoring.targets.iter().find_map(|target| {
                        match &target.intent {
                            MonitoringTargetIntent::Display { device_id, .. }
                                if Some(device_id)
                                    == app.snapshot.kraken_display.device_id.as_ref() =>
                            {
                                Some(target.actual_state)
                            }
                            _ => None,
                        }
                    });
                format!(
                    " LCD: {} ",
                    match state {
                        Some(MonitoringActualState::Pending) => "QUEUED (NOT UPLOADED)",
                        Some(MonitoringActualState::Unavailable) => "UNAVAILABLE",
                        Some(MonitoringActualState::ReviewRequired) => "REVIEW REQUIRED",
                        _ => app.snapshot.kraken_display.mode.title(),
                    }
                )
            } else {
                " LCD: STATUS UNKNOWN ".to_owned()
            },
            if app.host_status_trusted {
                theme.muted
            } else {
                theme.error
            },
        ));
    }
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let mut lines = Vec::new();
    for (device_index, device) in devices.iter().enumerate() {
        if device_index > 0 {
            lines.push(Line::styled("", theme.muted));
        }
        lines.extend(device_summary_lines(device, active_id, theme, inner.width));
    }

    frame.render_widget(
        Paragraph::new(lines)
            .style(theme.text)
            .wrap(Wrap { trim: true }),
        inner,
    );
}

fn device_summary_lines(
    device: &Device,
    active_id: Option<&DeviceId>,
    theme: Theme,
    available_width: u16,
) -> Vec<Line<'static>> {
    let is_active = active_id == Some(&device.id);
    let status = if available_width >= 30 {
        if device.online {
            "  ONLINE"
        } else {
            "  OFFLINE"
        }
    } else {
        ""
    };
    let marker = if !device.online {
        "× "
    } else if is_active {
        "▸ "
    } else {
        "• "
    };
    let mut lines = vec![Line::from(vec![
        Span::styled(
            marker,
            if device.online {
                theme.strong
            } else {
                theme.error
            },
        ),
        Span::styled(device.name.to_uppercase(), theme.strong),
        Span::styled(
            status,
            if device.online {
                theme.muted
            } else {
                theme.error
            },
        ),
    ])];

    for reading in &device.readings {
        match reading.kind {
            ReadingKind::Duty => continue,
            ReadingKind::Temperature => lines.push(Line::from(vec![
                Span::styled(
                    format!("  {:<10}", reading.label.to_uppercase()),
                    theme.muted,
                ),
                Span::styled(
                    format!("{:>5} {}", reading.formatted_value(), reading.unit),
                    theme.strong,
                ),
            ])),
            ReadingKind::Speed => {
                let stem = reading
                    .label
                    .strip_suffix(" speed")
                    .unwrap_or(&reading.label);
                let duty_label = format!("{stem} duty");
                let duty = device
                    .readings
                    .iter()
                    .find(|candidate| candidate.label == duty_label)
                    .map(|candidate| candidate.value.round() as u8);
                let duty_text = duty.map_or_else(|| " -- ".into(), |value| format!("{value:>3}%"));
                let stem = stem.to_uppercase();
                let short = stem.strip_prefix("IT8689 ").unwrap_or(&stem);
                let speed = format!("{:>4} ", reading.formatted_value());
                // Leave real separation between the fan label and its RPM.
                // On narrow panes shorten the label before dropping the gauge.
                let layouts = [
                    (stem.as_str(), 8, true),
                    (stem.as_str(), 5, false),
                    (short, 8, true),
                    (short, 5, false),
                    (short, 3, false),
                    (short, 0, false),
                ];
                let (label, gauge_width, show_unit) = layouts
                    .into_iter()
                    .find(|(label, gauge_width, show_unit)| {
                        4 + label.len()
                            + speed.len()
                            + *gauge_width
                            + duty_text.len()
                            + if *show_unit {
                                reading.unit.len().max(3) + 1
                            } else {
                                0
                            }
                            <= usize::from(available_width)
                    })
                    .unwrap_or((short, 0, false));
                let gauge = duty.map_or_else(
                    || "─".repeat(gauge_width),
                    |value| mini_gauge(value, gauge_width),
                );
                lines.push(Line::from(vec![
                    Span::styled(format!("  {label}  "), theme.muted),
                    Span::styled(speed, theme.strong),
                    Span::styled(
                        if show_unit {
                            format!("{:<3} ", reading.unit)
                        } else {
                            String::new()
                        },
                        theme.muted,
                    ),
                    Span::styled(gauge, theme.text),
                    Span::styled(duty_text, theme.muted),
                ]));
            }
            ReadingKind::ChannelCount => lines.push(Line::from(vec![
                Span::styled("  ARGB BUS   ", theme.muted),
                Span::styled(
                    format!("{} CHANNELS", reading.formatted_value()),
                    theme.strong,
                ),
            ])),
            ReadingKind::Mode => {
                let label = reading.label.to_uppercase();
                let stem = label.strip_suffix(" MODE").unwrap_or(&label);
                let short = stem.strip_prefix("IT8689 ").unwrap_or(stem);
                let mode = reading.formatted_value();
                let labels = [
                    format!("  {label}  "),
                    format!("  {stem}  "),
                    format!("  {short} MODE  "),
                    format!("  {short}  "),
                ];
                let label = labels
                    .into_iter()
                    .find(|label| label.len() + mode.len() <= usize::from(available_width))
                    .unwrap_or_else(|| "  MODE  ".into());
                lines.push(Line::from(vec![
                    Span::styled(label, theme.muted),
                    Span::styled(mode, theme.strong),
                ]));
            }
        }
    }

    lines
}

fn mini_gauge(duty: u8, width: usize) -> String {
    let filled = (usize::from(duty.min(100)) * width).div_ceil(100);
    format!("{}{}", "━".repeat(filled), "─".repeat(width - filled))
}

fn render_opt_in(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let block = Block::bordered()
        .border_style(theme.focused_border)
        .title(Line::styled(
            " CURVE LAB · START MONITORING · OPT IN ",
            theme.selected,
        ))
        .padding(Padding::uniform(2));
    let inner = block.inner(area);
    frame.render_widget(block, area);
    let action = if !app.snapshot_received {
        "Waiting for service snapshot; telemetry is read-only."
    } else if app.activation_busy {
        "Activation request in progress; waiting for service response."
    } else if app.activation_attempted {
        "Activation requested. Review outcomes and service snapshot; no automatic retry."
    } else {
        "ENTER  Activate available monitoring components"
    };
    frame.render_widget(Paragraph::new(vec![
        Line::styled("Monitoring has not been opted in on this service.", theme.strong),
        Line::raw(""),
        Line::raw("Enter writes available AIO pump/fan firmware curves and starts available motherboard fan control."),
        Line::raw("It can also select the current Kraken display mode. Service opt-in persists across boots."),
        Line::raw("Auto-resume on future restarts is a separate Settings preference."),
        Line::raw("Startup is read-only; Enter explicitly activates control. Settings offers a separate explicit board Stop."),
        Line::raw(""),
        Line::styled(action, theme.selected),
        Line::styled("O Settings   ? Help   Q Quit", theme.muted),
    ]).wrap(Wrap { trim: true }), inner);
}

fn render_settings(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let popup = centered_rect(76, 13, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(" SETTINGS ", theme.selected))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);
    frame.render_widget(
        Paragraph::new(vec![
            Line::raw(format!(
                "C  App: confirm firmware Apply   {}",
                if app.confirm_apply { "ON" } else { "OFF" }
            )),
            Line::raw(format!(
                "A  Service: auto-resume on FUTURE restarts   {}",
                if app.snapshot.monitoring.auto_resume {
                    "ON"
                } else {
                    "OFF"
                }
            )),
            Line::raw(if app.needs_opt_in() {
                "   Auto-resume becomes available after service opt-in."
            } else {
                "   Does not start or stop monitoring now."
            }),
            Line::raw("T  Stop motherboard fan control NOW (no firmware undo)"),
            Line::raw(""),
            Line::raw("Esc / O  Close settings"),
        ])
        .wrap(Wrap { trim: true }),
        inner,
    );
}

fn render_curve_editor(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    match app.editor_mode {
        EditorMode::Firmware => render_firmware_curve_editor(frame, area, app, theme),
        EditorMode::Host => render_host_curve_editor(frame, area, app, theme),
    }
}

fn firmware_monitoring_warning(app: &App) -> Option<&'static str> {
    let key = app.active_curve_key()?;
    let points = &app.active_channel()?.points;
    app.snapshot
        .monitoring
        .targets
        .iter()
        .find_map(|target| match &target.intent {
            MonitoringTargetIntent::AioCurve {
                device_id,
                channel_id,
                points: intent,
            } if *device_id == key.0 && *channel_id == key.1 && intent == points => {
                match target.actual_state {
                    MonitoringActualState::Pending => Some("QUEUED · NOT APPLIED"),
                    MonitoringActualState::Unavailable => Some("UNAVAILABLE"),
                    MonitoringActualState::ReviewRequired => Some("REVIEW REQUIRED"),
                    MonitoringActualState::Applied => None,
                }
            }
            _ => None,
        })
}

fn render_firmware_curve_editor(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let dirty = app.is_active_curve_dirty();
    let verified = app.active_curve_is_verified();
    let warning = firmware_monitoring_warning(app);
    let state = if let Some(warning) = warning {
        warning
    } else if app.active_channel().is_none() {
        " NO CHANNEL "
    } else {
        match (dirty, verified) {
            (true, false) => " MODIFIED · UNVERIFIED ",
            (true, true) => " MODIFIED ",
            (false, false) => " UNVERIFIED ",
            (false, true) => " APPLIED ",
        }
    };
    let title = Line::from(vec![
        Span::styled(" CURVE LAB ", theme.selected),
        Span::styled("  40-POINT FIRMWARE PROFILE ", theme.strong),
        Span::styled(
            state,
            if dirty || warning.is_some() {
                theme.error
            } else {
                theme.muted
            },
        ),
    ]);
    let instructions = curve_hints(false, true, true, area.width.saturating_sub(2), theme);
    let block = Block::bordered()
        .border_style(theme.focused_border)
        .title(title)
        .title_bottom(instructions)
        .padding(Padding::horizontal(1));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let Some(device) = app.active_device() else {
        frame.render_widget(
            Paragraph::new("No controllable cooling devices in this snapshot")
                .alignment(Alignment::Center)
                .style(theme.muted),
            inner,
        );
        return;
    };
    let Some(channel) = app.active_channel() else {
        return;
    };

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(2), Constraint::Min(7)])
        .split(inner);
    let context = Line::from(vec![
        Span::styled(" DEVICE ", theme.muted),
        Span::styled(device.name.to_uppercase(), theme.strong),
        Span::styled("   CHANNEL ", theme.muted),
        Span::styled(channel.name.to_uppercase(), theme.strong),
        Span::styled("   SOURCE ", theme.muted),
        Span::styled(channel.source.to_string(), theme.strong),
        Span::styled(
            format!(
                "   {:02}/{:02} ",
                app.selected_channel + 1,
                device.cooling_channels.len()
            ),
            theme.muted,
        ),
    ]);
    let profile_line = Line::from(vec![
        Span::styled(" PROFILE  ", theme.muted),
        Span::styled(app.active_profile_name().to_owned(), theme.selected),
        Span::styled("   P PICK", theme.muted),
    ]);
    frame.render_widget(Paragraph::new(vec![context, profile_line]), rows[0]);

    let columns = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Min(50), Constraint::Length(25)])
        .spacing(1)
        .split(rows[1]);
    frame.render_widget(
        CurveChart::new(&channel.points, app.selected_point, theme),
        columns[0],
    );
    render_curve_inspector(frame, columns[1], app, theme);
}

fn render_curve_inspector(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let block = Block::default()
        .borders(Borders::LEFT)
        .border_style(theme.border)
        .padding(Padding::left(2));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let Some(channel) = app.active_channel() else {
        return;
    };
    let Some(point) = app.selected_curve_point() else {
        return;
    };

    let warning = firmware_monitoring_warning(app);
    let state_style = if warning.is_some() || app.is_active_curve_dirty() {
        theme.error
    } else if app.active_curve_is_verified() {
        theme.muted
    } else {
        theme.strong
    };
    let lines = vec![
        Line::styled("SELECTED POINT", theme.muted),
        Line::from(vec![
            Span::styled(format!("{:02}°C", point.temperature), theme.strong),
            Span::styled("  →  ", theme.muted),
            Span::styled(format!("{:03}%", point.duty), theme.selected),
        ]),
        Line::styled(
            format!(
                "{:02} / {:02} BARS",
                app.selected_point + 1,
                channel.points.len()
            ),
            theme.muted,
        ),
        Line::styled("", theme.text),
        Line::from(vec![
            Span::styled("SOURCE   ", theme.muted),
            Span::styled(channel.source.to_string(), theme.strong),
        ]),
        Line::from(vec![
            Span::styled("TEMP     ", theme.muted),
            Span::styled("20–59 °C", theme.strong),
        ]),
        Line::from(vec![
            Span::styled("DUTY     ", theme.muted),
            Span::styled(
                format!("{}–{} %", channel.min_duty, channel.max_duty),
                theme.strong,
            ),
        ]),
        Line::from(vec![
            Span::styled("STATE    ", theme.muted),
            Span::styled(
                match (app.is_active_curve_dirty(), app.active_curve_is_verified()) {
                    _ if warning.is_some() => warning.unwrap(),
                    (true, false) => "MOD/UNKNOWN",
                    (true, true) => "MODIFIED",
                    (false, false) => "UNVERIFIED",
                    (false, true) => "APPLIED",
                },
                state_style,
            ),
        ]),
        Line::styled("", theme.text),
        Line::styled("SHIFT+↑/↓  ±5%", theme.muted),
        Line::styled("R           reset", theme.muted),
    ];
    frame.render_widget(Paragraph::new(lines).style(theme.text), inner);
}

fn host_state_label(app: &App) -> &'static str {
    if !app.host_status_trusted {
        "STATUS UNKNOWN"
    } else {
        match app.host_control_state {
            HostControlState::Disabled => "DISABLED",
            HostControlState::Available => "AVAILABLE",
            HostControlState::Running => "RUNNING",
            HostControlState::Restoring => "RESTORING",
            HostControlState::RestoreRequired => "RESTORE REQUIRED",
        }
    }
}

fn render_host_curve_editor(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let state = host_state_label(app);
    let dirty = app.host_policy_is_dirty();
    let title = Line::from(vec![
        Span::styled(" CURVE LAB ", theme.selected),
        Span::styled(
            app.selected_host_channel().map_or_else(
                || " MOTHERBOARD FAN CONTROL ".into(),
                |channel| format!(" {}-POINT FAN CURVE ", channel.curve.points.len()),
            ),
            theme.strong,
        ),
        Span::styled(
            format!(" {state}{} ", if dirty { " · MODIFIED" } else { "" }),
            if matches!(
                app.host_control_state,
                HostControlState::RestoreRequired | HostControlState::Disabled
            ) || !app.host_status_trusted
            {
                theme.error
            } else {
                theme.muted
            },
        ),
    ]);
    let enter = if !app.host_status_trusted {
        "STATUS UNKNOWN · O MENU"
    } else {
        match app.host_control_state {
            HostControlState::Running => "ENTER SCOPE · O MENU",
            HostControlState::Available if !app.host_editor.channels.is_empty() => "ENTER START",
            HostControlState::Available => "NO CHANNELS",
            HostControlState::Disabled => "CONTROL DISABLED",
            HostControlState::Restoring => "RESTORING",
            HostControlState::RestoreRequired => "RESTORE REQUIRED · O MENU",
        }
    };
    let has_channel = app.selected_host_channel().is_some();
    let can_edit = app.host_status_trusted
        && !app.host_operation_busy
        && matches!(
            app.host_control_state,
            HostControlState::Available | HostControlState::Running
        );
    let mut instructions = curve_hints(
        true,
        has_channel,
        can_edit,
        area.width.saturating_sub(2),
        theme,
    );
    if !has_channel {
        instructions
            .spans
            .push(Span::styled(format!(" {enter} "), theme.strong));
    }
    let block = Block::bordered()
        .border_style(theme.focused_border)
        .title(title)
        .title_bottom(instructions)
        .padding(Padding::horizontal(1));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let Some(channel) = app.selected_host_channel() else {
        let message = if !app.host_status_trusted {
            "Status unknown. Waiting for refresh."
        } else {
            match app.host_control_state {
                HostControlState::Disabled => {
                    "Motherboard control is disabled.\nNo fan channels are configured."
                }
                HostControlState::Available => "No motherboard fan channels available.",
                HostControlState::Running => "Running; channel details unavailable.",
                HostControlState::Restoring => "Restoring BIOS control…",
                HostControlState::RestoreRequired => "Recovery required. Check service status.",
            }
        };
        frame.render_widget(
            Paragraph::new(message)
                .alignment(Alignment::Center)
                .wrap(Wrap { trim: true })
                .style(theme.muted),
            inner,
        );
        return;
    };
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(2), Constraint::Min(7)])
        .split(inner);
    let profile_line = Line::from(vec![
        Span::styled(" PROFILE  ", theme.muted),
        Span::styled(
            app.active_host_profile_name()
                .unwrap_or_else(|| "Custom".into())
                .to_owned(),
            theme.selected,
        ),
        Span::styled("   P PICK", theme.muted),
    ]);
    let context = vec![
        Line::from(vec![
            Span::styled(" CHANNEL ", theme.muted),
            Span::styled(channel.capability.name.to_uppercase(), theme.strong),
            Span::styled("   SOURCE ", theme.muted),
            Span::styled(channel.curve.source.to_string(), theme.strong),
            Span::styled(
                format!(
                    "   {:02}/{:02} ",
                    app.host_editor.selected_channel + 1,
                    app.host_editor.channels.len()
                ),
                theme.muted,
            ),
        ]),
        profile_line,
    ];
    frame.render_widget(Paragraph::new(context), rows[0]);

    let columns = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Min(50), Constraint::Length(25)])
        .spacing(1)
        .split(rows[1]);
    frame.render_widget(
        CurveChart::new(&channel.curve.points, app.host_editor.selected_point, theme),
        columns[0],
    );

    let inspector_block = Block::default()
        .borders(Borders::LEFT)
        .border_style(theme.border)
        .padding(Padding::left(2));
    let inspector = inspector_block.inner(columns[1]);
    frame.render_widget(inspector_block, columns[1]);
    let Some(point) = app.selected_host_point() else {
        return;
    };
    let temperature = app
        .host_source_temperature_celsius()
        .map_or_else(|| "--.- °C".into(), |value| format!("{value:.1} °C"));
    let duty = app
        .host_target_duty()
        .map_or_else(|| "---%".into(), |value| format!("{value:03}%"));
    let lines = vec![
        Line::styled("SELECTED POINT", theme.muted),
        Line::from(vec![
            Span::styled(
                format!("{}°C", point.temperature_millidegrees / 1_000),
                theme.strong,
            ),
            Span::styled("  →  ", theme.muted),
            Span::styled(format!("{:03}%", point.duty_percent), theme.selected),
        ]),
        Line::styled(
            format!(
                "{:02} / {:02} BARS",
                app.host_editor.selected_point + 1,
                channel.curve.points.len()
            ),
            theme.muted,
        ),
        Line::styled("", theme.text),
        Line::from(vec![
            Span::styled("STATE    ", theme.muted),
            Span::styled(state, theme.strong),
        ]),
        Line::from(vec![
            Span::styled("MINIMUM  ", theme.muted),
            Span::styled(
                format!("{}%", channel.capability.minimum_duty_percent),
                theme.strong,
            ),
        ]),
        Line::from(vec![
            Span::styled(
                if app.host_status_trusted
                    && app.host_control_state == HostControlState::Running
                    && !app.host_editor.selected_is_dirty()
                {
                    "TARGET   "
                } else if app.host_editor.selected_is_dirty() {
                    "DRAFT    "
                } else {
                    "PREVIEW  "
                },
                theme.muted,
            ),
            Span::styled(format!("{temperature} / {duty}"), theme.strong),
        ]),
        Line::styled(if can_edit { "SHIFT+↑/↓  ±5%" } else { "" }, theme.muted),
        Line::styled(
            if !app.host_status_trusted
                || app.host_control_state == HostControlState::RestoreRequired
            {
                "O SETTINGS"
            } else {
                enter
            },
            theme.selected,
        ),
    ];
    frame.render_widget(Paragraph::new(lines).style(theme.text), inspector);
}

fn curve_hints(
    host: bool,
    has_channel: bool,
    can_edit: bool,
    width: u16,
    theme: Theme,
) -> Line<'static> {
    let mut shortcuts = Vec::new();
    if has_channel {
        shortcuts.push(("←/→", "point", "point"));
        if can_edit {
            shortcuts.push(("↑/↓", "duty", "duty"));
        }
    }
    shortcuts.extend([("Tab", "view", "view"), ("P", "profiles", "profiles")]);
    if has_channel {
        shortcuts.push(("C", "channel", "chan"));
        if can_edit {
            shortcuts.push(("R", "reset", "reset"));
        }
    }
    if host {
        if has_channel && can_edit {
            shortcuts.push(("S", "source", "src"));
            shortcuts.push(("Enter", "apply", "apply"));
        }
        shortcuts.push(("O", "settings", "settings"));
    } else {
        shortcuts.push(("Enter", "apply", "apply"));
    }
    let build = |compact| {
        let mut spans = Vec::new();
        for (key, label, short) in &shortcuts {
            spans.push(Span::styled(format!(" {key} "), theme.strong));
            spans.push(Span::styled(
                format!(
                    "{}{}",
                    if compact { short } else { label },
                    if compact { "" } else { " " }
                ),
                theme.muted,
            ));
        }
        Line::from(spans)
    };
    let full = build(false);
    if full.width() <= usize::from(width) {
        full
    } else {
        build(true)
    }
}

fn render_footer(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let columns = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Min(24), Constraint::Length(16)])
        .split(Rect::new(area.x, area.y, area.width, area.height.min(2)));
    let style = match app.status.kind {
        StatusKind::Info => theme.muted,
        StatusKind::Success => theme.strong,
        StatusKind::Error => theme.error,
    };
    let marker = match app.status.kind {
        StatusKind::Info => "◇",
        StatusKind::Success => "◆",
        StatusKind::Error => "!",
    };
    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(format!(" {marker} "), style),
            if app.host_status_trusted {
                Span::styled(app.status.text.clone(), style)
            } else {
                Span::styled(format!("STATUS UNKNOWN · {}", app.status.text), theme.error)
            },
        ]))
        .block(
            Block::default()
                .borders(Borders::TOP)
                .border_style(theme.border),
        ),
        columns[0],
    );
    frame.render_widget(
        Paragraph::new(Line::styled("? HELP  Q QUIT", theme.muted))
            .alignment(Alignment::Right)
            .block(
                Block::default()
                    .borders(Borders::TOP)
                    .border_style(theme.border),
            ),
        columns[1],
    );
    if let Some(error) = &app.snapshot.kraken_display.last_error
        && area.height >= 2
    {
        frame.render_widget(
            Paragraph::new(format!(" ! Last LCD error: {error}")).style(theme.error),
            Rect::new(area.x, area.y + 1, area.width, 1),
        );
    }
    if let Some(error) = &app.snapshot.host_control.last_error
        && area.height >= 3
    {
        // Controller faults outlive transient navigation/apply messages. Keep
        // one full-width row only while an actual service error is present.
        frame.render_widget(
            Paragraph::new(format!(" ! Last fan error: {error}")).style(theme.error),
            Rect::new(area.x, area.y + 2, area.width, 1),
        );
    }
}

fn render_display_picker(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let popup = centered_rect(64, 17, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(" KRAKEN 2023 DISPLAY ", theme.selected))
        .title_bottom(Line::styled(
            " ↑/↓ select   Enter apply   Esc close ",
            theme.muted,
        ))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);
    let mut lines = vec![
        Line::styled(
            "Built-in liquid is device-rendered; other faces update every 2 seconds.",
            theme.muted,
        ),
        Line::raw(""),
    ];
    for (index, mode) in KrakenDisplayMode::ALL.iter().enumerate() {
        lines.push(Line::from(vec![
            Span::styled(
                if index == app.display_cursor {
                    "▸ "
                } else {
                    "  "
                },
                theme.strong,
            ),
            Span::styled(
                mode.title(),
                if index == app.display_cursor {
                    theme.selected
                } else {
                    theme.text
                },
            ),
            Span::styled(
                if *mode == app.snapshot.kraken_display.mode {
                    "  SELECTED"
                } else {
                    ""
                },
                theme.muted,
            ),
        ]));
    }
    if app.display_operation_busy {
        lines.push(Line::styled("Display change in progress…", theme.muted));
    }
    if let Some(error) = &app.snapshot.kraken_display.last_error {
        lines.push(Line::styled(format!("Last error: {error}"), theme.error));
    }
    frame.render_widget(Paragraph::new(lines).wrap(Wrap { trim: true }), inner);
}

fn render_profile_picker(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let desired_height = (app.profiles.len() as u16 + 6).clamp(10, 22);
    let popup = centered_rect(66, desired_height, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(" COOLING PROFILE LIBRARY ", theme.selected))
        .title_bottom(Line::from(vec![
            Span::styled(" ↑/↓ ", theme.strong),
            Span::styled("select  ", theme.muted),
            Span::styled(" Enter ", theme.strong),
            Span::styled("use + apply  ", theme.muted),
            Span::styled(" R ", theme.strong),
            Span::styled("rename custom  ", theme.muted),
            Span::styled(" Esc ", theme.strong),
            Span::styled("close ", theme.muted),
        ]))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);

    let visible = usize::from(inner.height.saturating_sub(2));
    let start = app.profile_cursor.saturating_sub(visible.saturating_sub(1));
    let end = (start + visible).min(app.profiles.len());
    let mut lines = vec![Line::from(vec![
        Span::styled("PROFILE", theme.muted),
        Span::styled("                                      TYPE", theme.muted),
    ])];
    for (index, profile) in app.profiles[start..end].iter().enumerate() {
        let absolute_index = start + index;
        let selected = absolute_index == app.profile_cursor;
        lines.push(Line::from(vec![
            Span::styled(if selected { "▸ " } else { "  " }, theme.strong),
            Span::styled(
                format!("{:<42}", profile.name),
                if selected { theme.selected } else { theme.text },
            ),
            Span::styled(
                profile.kind.label(),
                if profile.kind == ProfileKind::Custom {
                    theme.strong
                } else {
                    theme.muted
                },
            ),
        ]));
    }
    if app.profiles.len() > visible {
        lines.push(Line::styled(
            format!(
                "Showing {}–{} of {} profiles",
                start + 1,
                end,
                app.profiles.len()
            ),
            theme.muted,
        ));
    }
    frame.render_widget(Paragraph::new(lines), inner);
}

fn render_host_profile_picker(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let profiles = app.host_profiles();
    let popup = centered_rect(66, (profiles.len() as u16 + 8).clamp(11, 22), area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(" MOTHERBOARD PROFILES ", theme.selected))
        .title_bottom(Line::from(vec![
            Span::styled(" ↑/↓ ", theme.strong),
            Span::styled("select  ", theme.muted),
            Span::styled(" Enter ", theme.strong),
            Span::styled(
                if app.host_control_state == HostControlState::Running {
                    "load / choose scope  "
                } else {
                    "load  "
                },
                theme.muted,
            ),
            Span::styled(" R ", theme.strong),
            Span::styled("rename custom  ", theme.muted),
            Span::styled(" Esc ", theme.strong),
            Span::styled("close ", theme.muted),
        ]))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);
    let mut lines = vec![Line::styled("22–100 °C", theme.muted)];
    if let Some(channel) = app.selected_host_channel() {
        lines.push(Line::styled(
            format!(
                "{} · {} · min {}%",
                channel.capability.name,
                channel.curve.source,
                channel.capability.minimum_duty_percent
            ),
            theme.muted,
        ));
    }
    lines.push(Line::raw(""));
    let visible = usize::from(inner.height.saturating_sub(lines.len() as u16 + 1)).max(1);
    let start = app.profile_cursor.saturating_sub(visible.saturating_sub(1));
    let end = (start + visible).min(profiles.len());
    for (index, profile) in profiles[start..end].iter().enumerate() {
        let selected = start + index == app.profile_cursor;
        lines.push(Line::from(vec![
            Span::styled(if selected { "▸ " } else { "  " }, theme.strong),
            Span::styled(
                profile.name.clone(),
                if selected { theme.selected } else { theme.text },
            ),
            Span::styled(
                if start + index >= 3 {
                    "  CUSTOM"
                } else {
                    "  BUILT-IN"
                },
                theme.muted,
            ),
        ]));
    }
    if profiles.len() > visible {
        lines.push(Line::styled(
            format!(
                "Showing {}–{} of {} profiles",
                start + 1,
                end,
                profiles.len()
            ),
            theme.muted,
        ));
    }
    frame.render_widget(Paragraph::new(lines), inner);
}

fn render_host_apply_scope(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let Some(scope) = &app.host_apply_scope else {
        return;
    };
    let popup = centered_rect(74, 13, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(
            " APPLY MOTHERBOARD FAN CURVE ",
            theme.selected,
        ))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);
    let lines = vec![
        Line::styled(
            format!(
                "From {} ({}) · source {}",
                scope.selected.name, scope.selected.channel_id.0, scope.curve.source
            ),
            theme.strong,
        ),
        Line::styled("All copies this curve and CPU/GPU/MAX source.", theme.muted),
        Line::styled("Each group's minimum duty is respected.", theme.muted),
        Line::raw(""),
        Line::from(vec![
            Span::styled(if scope.all_groups { "  " } else { "▸ " }, theme.strong),
            Span::styled(
                format!(
                    "Only {} ({})",
                    scope.selected.name, scope.selected.channel_id.0
                ),
                if scope.all_groups {
                    theme.text
                } else {
                    theme.selected
                },
            ),
        ]),
        Line::from(vec![
            Span::styled(if scope.all_groups { "▸ " } else { "  " }, theme.strong),
            Span::styled(
                format!("All fan groups ({})", app.host_editor.channels.len()),
                if scope.all_groups {
                    theme.selected
                } else {
                    theme.text
                },
            ),
        ]),
        Line::raw(""),
        Line::styled(
            "↑/↓ Home/End select   Enter apply   Esc cancel   O settings   Q quit",
            theme.muted,
        ),
    ];
    frame.render_widget(Paragraph::new(lines).wrap(Wrap { trim: true }), inner);
}

fn render_profile_name_input(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let popup = centered_rect(62, 9, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(" RENAME CUSTOM PROFILE ", theme.selected))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);

    let source = if app.editor_mode == EditorMode::Host {
        app.selected_host_channel()
            .map_or("—", |channel| channel.capability.name.as_str())
    } else {
        app.active_channel()
            .map_or("—", |channel| channel.name.as_str())
    };
    let input = if app.profile_name_input.is_empty() {
        Span::styled("type a new name", theme.muted)
    } else {
        Span::styled(&app.profile_name_input, theme.strong)
    };
    let lines = vec![
        Line::from(vec![
            Span::styled("NAME  ", theme.muted),
            input,
            Span::styled("▌", theme.strong),
        ]),
        Line::styled("", theme.text),
        Line::from(vec![
            Span::styled("SOURCE  ", theme.muted),
            Span::styled(source.to_owned(), theme.strong),
            Span::styled(" · rename only · no hardware write", theme.muted),
        ]),
        Line::styled("Enter rename   Backspace edit   Esc cancel", theme.muted),
    ];
    frame.render_widget(Paragraph::new(lines), inner);
}

fn render_apply_confirmation(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let popup = centered_rect(70, 13, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.error)
        .title(Line::styled(
            " CONFIRM FIRMWARE CURVE WRITE ",
            theme.selected,
        ))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);

    let device = app
        .active_device()
        .map_or("—", |device| device.name.as_str());
    let channel = app
        .active_channel()
        .map_or("—", |channel| channel.name.as_str());
    let lines = vec![
        Line::styled(
            "This will replace the device's stored cooling curve.",
            theme.strong,
        ),
        Line::styled(
            "The previous firmware curve cannot be read back or restored automatically.",
            theme.error,
        ),
        Line::styled("", theme.text),
        Line::from(vec![
            Span::styled("DEVICE   ", theme.muted),
            Span::styled(device.to_uppercase(), theme.strong),
        ]),
        Line::from(vec![
            Span::styled("CHANNEL  ", theme.muted),
            Span::styled(channel.to_owned(), theme.strong),
            Span::styled("   PROFILE  ", theme.muted),
            Span::styled(app.active_profile_name().to_owned(), theme.strong),
        ]),
        Line::styled("", theme.text),
        Line::from(vec![
            Span::styled("Y", theme.selected),
            Span::styled(" apply curve     ", theme.strong),
            Span::styled("N / Esc", theme.selected),
            Span::styled(" cancel", theme.strong),
        ]),
    ];
    frame.render_widget(Paragraph::new(lines).wrap(Wrap { trim: true }), inner);
}

fn render_help(frame: &mut Frame, area: Rect, app: &App, theme: Theme) {
    let popup = centered_rect(70, 22, area);
    frame.render_widget(Clear, popup);
    let block = Block::bordered()
        .border_type(BorderType::Double)
        .border_style(theme.focused_border)
        .title(Line::styled(" KEYBOARD / CONTROL MAP ", theme.selected))
        .padding(Padding::uniform(1));
    let inner = block.inner(popup);
    frame.render_widget(block, popup);

    let help = vec![
        help_line("← / →", "Select the previous / next temperature bar", theme),
        help_line("↑ / ↓", "Adjust selected duty by 1%", theme),
        help_line("Shift+↑ / ↓", "Adjust selected duty by 5%", theme),
        help_line(
            "Curve ↑ / ↓",
            "Hotter follow raises; cooler follow reductions",
            theme,
        ),
        help_line("Home / End", "Jump to first / last point", theme),
        help_line(
            "Tab / Shift+Tab",
            "Cycle firmware devices / motherboard fans",
            theme,
        ),
        help_line("C or [ / ]", "Select cooling channel", theme),
        help_line("S", "Host: cycle source CPU/GPU/MAX", theme),
        help_line("Enter", "Opt in to monitoring / apply changes", theme),
        help_line("P", "Pick: host loads draft; firmware applies", theme),
        help_line("D", "Select Kraken 2023 display face", theme),
        help_line(
            "O",
            "Settings: app preference, future auto-resume, stop board",
            theme,
        ),
        help_line("R", "Reset curve; in P rename custom profile", theme),
        help_line("? / Esc", "Close this help", theme),
        help_line("Q", "Quit (fan control continues)", theme),
        Line::styled("", theme.text),
        Line::styled(
            if app.backend_name == "DEMO" {
                "DEMO MODE · devices and writes are simulated"
            } else {
                "Startup is read-only until Enter opt-in; applies write firmware"
            },
            theme.muted,
        ),
    ];
    frame.render_widget(Paragraph::new(help), inner);
}

fn help_line(key: &str, description: &str, theme: Theme) -> Line<'static> {
    Line::from(vec![
        Span::styled(format!("{key:<17}"), theme.strong),
        Span::styled(description.to_owned(), theme.text),
    ])
}

fn centered_rect(width: u16, height: u16, area: Rect) -> Rect {
    let width = width.min(area.width.saturating_sub(4));
    let height = height.min(area.height.saturating_sub(2));
    Rect::new(
        area.x + area.width.saturating_sub(width) / 2,
        area.y + area.height.saturating_sub(height) / 2,
        width,
        height,
    )
}

fn render_too_small(frame: &mut Frame, area: Rect, theme: Theme) {
    let message = vec![
        Line::styled("NZXT//CAM TUI", theme.strong),
        Line::styled("", theme.text),
        Line::styled(
            "Terminal too small for the 40-point curve editor.",
            theme.text,
        ),
        Line::styled(
            format!(
                "Current: {}×{}   Required: at least {}×{}",
                area.width, area.height, MIN_WIDTH, MIN_HEIGHT
            ),
            theme.muted,
        ),
        Line::styled("Resize the terminal, or press Q to quit.", theme.muted),
    ];
    frame.render_widget(
        Paragraph::new(message)
            .alignment(Alignment::Center)
            .block(Block::bordered().border_style(theme.border))
            .wrap(Wrap { trim: true }),
        area,
    );
}

#[cfg(test)]
mod tests {
    use ratatui::{Terminal, backend::TestBackend};

    use super::*;
    use crate::backend::{DemoBackend, HardwareBackend};

    fn dashboard_rows(app: &App, width: u16, height: u16) -> Vec<String> {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        terminal.draw(|frame| draw(frame, app)).unwrap();
        let buffer = terminal.backend().buffer();
        (0..height)
            .map(|y| {
                (0..width)
                    .map(|x| buffer[(x, y)].symbol())
                    .collect::<String>()
            })
            .collect()
    }

    #[test]
    fn queued_firmware_intent_does_not_render_as_applied() {
        use crate::model::{
            MonitoringActualState, MonitoringTargetIntent, MonitoringTargetSnapshot,
        };
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.monitoring.opted_in = true;
        snapshot.monitoring.targets.push(MonitoringTargetSnapshot {
            intent: MonitoringTargetIntent::AioCurve {
                device_id: snapshot.devices[0].id.clone(),
                channel_id: snapshot.devices[0].cooling_channels[0].id.clone(),
                points: snapshot.devices[0].cooling_channels[0].points.clone(),
            },
            actual_state: MonitoringActualState::Pending,
            last_error: None,
        });
        let app = App::new(snapshot, "MONITORING SERVICE");
        let rendered = dashboard_rows(&app, 120, 38).join("\n");
        assert!(rendered.contains("QUEUED · NOT APPLIED"));
        assert!(!rendered.contains("STATE    APPLIED"));
    }

    #[test]
    fn pending_monitoring_display_does_not_claim_selected_face_is_applied() {
        use crate::model::{
            MonitoringActualState, MonitoringTargetIntent, MonitoringTargetSnapshot,
        };
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.monitoring.opted_in = true;
        snapshot.kraken_display.mode = KrakenDisplayMode::Cpu;
        snapshot.monitoring.targets.push(MonitoringTargetSnapshot {
            intent: MonitoringTargetIntent::Display {
                device_id: snapshot.kraken_display.device_id.clone().unwrap(),
                mode: KrakenDisplayMode::Cpu,
            },
            actual_state: MonitoringActualState::Pending,
            last_error: None,
        });
        let app = App::new(snapshot, "MONITORING SERVICE");
        let rendered = dashboard_rows(&app, 120, 38).join("\n");
        assert!(rendered.contains("LCD: QUEUED"));
        assert!(!rendered.contains("LCD: CPU"));
    }

    #[test]
    fn gated_service_keeps_overview_and_replaces_only_curve_lab_with_opt_in() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot.clone(), "MONITORING SERVICE");
        let before = dashboard_rows(&app, 120, 38).join("\n");
        assert!(before.contains("START MONITORING · OPT IN"));
        assert!(before.contains("Waiting for service snapshot"));
        assert!(before.contains("PUMP"));
        assert!(!before.contains("40-POINT FIRMWARE PROFILE"));
        app.update_telemetry(snapshot);
        let after = dashboard_rows(&app, 120, 38).join("\n");
        assert!(after.contains("ENTER  Activate available monitoring components"));
        app.modal = Some(Modal::Settings);
        let settings = dashboard_rows(&app, 120, 38).join("\n");
        assert!(settings.contains("auto-resume on FUTURE restarts"));
        assert!(settings.contains("Stop motherboard fan control NOW"));
        app.modal = None;
        app.snapshot.monitoring.opted_in = true;
        assert!(
            dashboard_rows(&app, 120, 38)
                .join("\n")
                .contains("40-POINT FIRMWARE PROFILE")
        );
    }

    #[test]
    fn display_picker_shows_all_modes_and_current_service_selection() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.modal = Some(Modal::KrakenDisplay);
        let rendered = dashboard_rows(&app, 120, 38).join("\n");
        assert!(rendered.contains("KRAKEN 2023 DISPLAY"));
        assert!(rendered.contains("Built-in liquid"));
        assert!(rendered.contains("SELECTED"));
        for mode in KrakenDisplayMode::ALL {
            assert!(rendered.contains(mode.title()), "missing {mode:?}");
        }
    }

    #[test]
    fn disconnected_service_never_presents_stale_lcd_mode_as_current() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "MONITORING SERVICE");
        app.snapshot.kraken_display.last_error = Some("upload failed".into());
        app.mark_host_status_unknown();
        let rendered = dashboard_rows(&app, MIN_WIDTH, MIN_HEIGHT).join("\n");
        assert!(rendered.contains("LCD: STATUS UNKNOWN"));
        assert!(rendered.contains("Last LCD error: upload failed"));
        assert!(!rendered.contains("LCD: Built-in liquid"));
    }

    #[test]
    fn main_header_footer_help_and_curve_shortcuts_are_consistent() {
        use crossterm::event::KeyCode;

        let mut backend = DemoBackend::new();
        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            for demo in [false, true] {
                let mut snapshot = backend.refresh().unwrap();
                if !demo {
                    snapshot.monitoring.opted_in = true;
                }
                let mut app = App::new(snapshot, if demo { "DEMO" } else { "MONITORING SERVICE" });
                for host in [false, true] {
                    if host {
                        app.handle_key_event(KeyCode::BackTab.into());
                    }
                    let rows = dashboard_rows(&app, width, height);
                    let header = &rows[0];
                    assert_eq!(header.contains("DEMO"), demo);
                    assert!(!header.contains("SERVICE"));
                    assert!(!header.contains("READINGS"));
                    assert!(!header.contains("DEVICES"));
                    assert!(rows[height as usize - 1].contains("? HELP  Q QUIT"));
                    assert!(!rows[height as usize - 1].contains("MOD"));
                    let footer = rows
                        .iter()
                        .find(|row| {
                            row.contains("←/→ point")
                                && row.contains("R reset")
                                && row.contains("Enter apply")
                                && row.contains("Tab view")
                                && row.contains("P profiles")
                                && row.contains("C ")
                        })
                        .unwrap_or_else(|| {
                            panic!("missing complete curve hints: {}", rows.join("\n"))
                        });
                    let mut last = 0;
                    let shortcuts: &[&str] = if host {
                        &["←/→", "↑/↓", "Tab", "P", "C", "R", "S", "Enter", "O"]
                    } else {
                        &["←/→", "↑/↓", "Tab", "P", "C", "R", "Enter"]
                    };
                    for shortcut in shortcuts {
                        let offset = footer[last..].find(shortcut).unwrap_or_else(|| {
                            panic!("{shortcut} out of order or clipped: {footer}")
                        });
                        last += offset + shortcut.len();
                    }
                }
                app.handle_key_event(KeyCode::Char('?').into());
                let help = dashboard_rows(&app, width, height).join("\n");
                assert!(help.contains("Opt in to monitoring / apply changes"));
                assert!(help.contains("Reset curve; in P rename custom profile"));
                assert!(!help.contains("LIVE MODE"));
            }
        }
    }

    #[test]
    fn host_hints_do_not_advertise_apply_when_control_is_unavailable() {
        use crossterm::event::KeyCode;

        let mut backend = DemoBackend::new();
        for state in [
            HostControlState::Disabled,
            HostControlState::Restoring,
            HostControlState::RestoreRequired,
        ] {
            let mut app = App::new(backend.refresh().unwrap(), "DEMO");
            app.handle_key_event(KeyCode::BackTab.into());
            app.host_control_state = state;
            let rows = dashboard_rows(&app, MIN_WIDTH, MIN_HEIGHT);
            let hints = rows.iter().find(|row| row.contains("←/→ point")).unwrap();
            assert!(hints.contains("O settings"), "{state:?}: {hints}");
            assert!(!hints.contains("Enter apply"), "{state:?}: {hints}");
            for blocked in ["↑/↓ duty", "R reset", "S source", "S src"] {
                assert!(!hints.contains(blocked), "{state:?}: {hints}");
            }
            assert!(!rows.join("\n").contains("SHIFT+↑/↓"));
        }
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.handle_key_event(KeyCode::BackTab.into());
        app.mark_host_status_unknown();
        let rows = dashboard_rows(&app, MIN_WIDTH, MIN_HEIGHT);
        let hints = rows.iter().find(|row| row.contains("←/→ point")).unwrap();
        assert!(!hints.contains("Enter apply"));
        assert!(!hints.contains("↑/↓ duty"));
        assert!(!hints.contains("R reset"));
        assert!(!hints.contains("S source"));
        assert!(hints.contains("O settings"));
        assert!(!rows.join("\n").contains("SHIFT+↑/↓"));
        app.host_status_trusted = true;
        app.host_operation_busy = true;
        let rows = dashboard_rows(&app, MIN_WIDTH, MIN_HEIGHT);
        let hints = rows.iter().find(|row| row.contains("←/→ point")).unwrap();
        assert!(!hints.contains("Enter apply"));
        assert!(!hints.contains("R reset"));
        assert!(!hints.contains("S src"));
    }

    #[test]
    fn firmware_dashboard_shows_only_working_profile_and_picker_shows_library() {
        use crossterm::event::KeyCode;

        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let mut backend = DemoBackend::new();
            let mut app = App::new(backend.refresh().unwrap(), "DEMO");
            let rows = dashboard_rows(&app, width, height);
            let dashboard = rows.join("\n");
            assert!(
                dashboard.contains("PROFILE  Custom   P PICK"),
                "{dashboard}"
            );
            assert!(!dashboard.contains("Silent"));
            assert!(!dashboard.contains("Progressive"));
            assert!(!dashboard.contains("Performance"));
            assert!(
                rows.iter().any(|row| row.contains("Tab view")
                    && row.contains("P profiles")
                    && row.contains("R reset")
                    && row.contains("Enter apply")),
                "{dashboard}"
            );
            assert!(rows.iter().any(|row| row.contains("? HELP  Q QUIT")));
            assert!(!dashboard.contains("TAB VIEW  P PROFILES"));
            assert!(!dashboard.contains("H MODE"));
            assert!(!dashboard.contains("H firmware"));

            assert!(app.handle_key_event(KeyCode::Char('p').into()).is_none());
            let picker = dashboard_rows(&app, width, height).join("\n");
            assert!(picker.contains("Silent"));
            assert!(picker.contains("Progressive"));
            assert!(picker.contains("Performance"));
            app.handle_key_event(KeyCode::Esc.into());
            assert_eq!(app.modal, None);
            assert!(
                dashboard_rows(&app, width, height)
                    .join("\n")
                    .contains("PROFILE  Custom")
            );

            app.handle_key_event(KeyCode::Char('p').into());
            app.handle_key_event(KeyCode::Home.into());
            let _ = app.handle_key_event(KeyCode::Enter.into());
            assert_eq!(app.active_profile_name(), "Silent");
            let selected = dashboard_rows(&app, width, height).join("\n");
            assert!(selected.contains("PROFILE  Silent   P PICK"));
            assert!(!selected.contains("Progressive"));
            assert!(!selected.contains("Performance"));
            app.handle_key_event(KeyCode::Up.into());
            assert_eq!(app.active_profile_name(), "Custom");
            let edited = dashboard_rows(&app, width, height).join("\n");
            assert!(edited.contains("PROFILE  Custom   P PICK"));
            assert!(!edited.contains("SILENT*"));
            assert!(!edited.contains("PROFILE  Silent"));
            app.handle_key_event(KeyCode::Char('p').into());
            let picker = dashboard_rows(&app, width, height).join("\n");
            assert!(picker.contains("Silent"));
            assert!(picker.contains("Progressive"));
            assert!(picker.contains("Performance"));
        }
    }

    #[test]
    fn host_dashboard_shows_only_working_profile_and_picker_shows_presets() {
        use crossterm::event::KeyCode;

        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let mut backend = DemoBackend::new();
            let mut app = App::new(backend.refresh().unwrap(), "DEMO");
            app.handle_key_event(KeyCode::BackTab.into());
            assert_eq!(app.editor_mode, EditorMode::Host);
            let current = app
                .active_host_profile_name()
                .unwrap_or_else(|| "Custom".into());
            let rows = dashboard_rows(&app, width, height);
            let dashboard = rows.join("\n");
            assert!(dashboard.contains(&format!("PROFILE  {}   P PICK", current)));
            assert!(
                rows.iter().any(|row| row.contains("Tab view")
                    && row.contains("P profiles")
                    && row.contains("R reset")
                    && row.contains("Enter apply")
                    && row.contains("O settings")),
                "{dashboard}"
            );
            assert!(rows.iter().any(|row| row.contains("ENTER START")));
            assert!(rows.iter().any(|row| row.contains("? HELP  Q QUIT")));
            assert!(!dashboard.contains("TAB VIEW  P PROFILES"));
            assert!(!dashboard.contains("H MODE"));
            assert!(!dashboard.contains("H firmware"));
            for preset in ["Silent", "Progressive", "Performance"] {
                if current != preset {
                    assert!(!dashboard.contains(preset));
                }
            }
            app.handle_key_event(KeyCode::Char('p').into());
            let picker = dashboard_rows(&app, width, height).join("\n");
            for preset in ["Silent", "Progressive", "Performance"] {
                assert!(picker.contains(preset));
            }
            app.handle_key_event(KeyCode::Esc.into());
            assert_eq!(app.modal, None);
            assert!(
                dashboard_rows(&app, width, height)
                    .join("\n")
                    .contains(&format!("PROFILE  {}   P PICK", current))
            );

            app.handle_key_event(KeyCode::Char('p').into());
            app.handle_key_event(KeyCode::Home.into());
            app.handle_key_event(KeyCode::Down.into());
            assert!(app.handle_key_event(KeyCode::Enter.into()).is_none());
            assert_eq!(
                app.active_host_profile_name().as_deref(),
                Some("Progressive")
            );
            let selected = dashboard_rows(&app, width, height).join("\n");
            assert!(selected.contains("PROFILE  Progressive   P PICK"));
            assert!(!selected.contains("Silent"));
            assert!(!selected.contains("Performance"));
            app.handle_key_event(KeyCode::Up.into());
            assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
            let edited = dashboard_rows(&app, width, height).join("\n");
            assert!(edited.contains("PROFILE  Custom   P PICK"));
            assert!(!edited.contains("PROFILE  Progressive"));
            app.handle_key_event(KeyCode::Char('p').into());
            let picker = dashboard_rows(&app, width, height).join("\n");
            for preset in ["Silent", "Progressive", "Performance"] {
                assert!(picker.contains(preset));
            }
        }
    }

    #[test]
    fn ambiguous_host_preset_is_labeled_custom_instead_of_guessing() {
        use crossterm::event::KeyCode;

        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.host_control.channels[0].minimum_duty_percent = 100;
        let mut app = App::new(snapshot, "DEMO");
        app.handle_key_event(KeyCode::BackTab.into());
        app.handle_key_event(KeyCode::Char('p').into());
        app.handle_key_event(KeyCode::Home.into());
        app.handle_key_event(KeyCode::Enter.into());
        assert_eq!(app.active_host_profile_name(), None);
        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let dashboard = dashboard_rows(&app, width, height).join("\n");
            assert!(dashboard.contains("PROFILE  Custom   P PICK"));
            assert!(!dashboard.contains("PROFILE  Silent"));
        }
    }

    #[test]
    fn dashboard_renders_primary_sections_and_forty_point_editor() {
        let mut backend = DemoBackend::new();
        let app = App::new(backend.refresh().unwrap(), backend.name());
        let test_backend = TestBackend::new(120, 38);
        let mut terminal = Terminal::new(test_backend).unwrap();

        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let buffer = terminal.backend().buffer();
        let rendered = buffer
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains("AIO / LIQUID"));
        assert!(rendered.contains("FANS / AIRFLOW"));
        assert!(rendered.contains("USB / LIGHTING"));
        assert!(rendered.contains("CURVE LAB"));
        assert!(rendered.contains("40-POINT FIRMWARE PROFILE"));
        assert!(rendered.contains("PROFILE  Custom   P PICK"));
        assert!(!rendered.contains("Silent"));
        assert!(!rendered.contains("Progressive"));
        assert!(!rendered.contains("Performance"));
        assert!(rendered.contains("Kraken 2023".to_uppercase().as_str()));
        assert!(rendered.contains('█'));
    }

    #[test]
    fn host_editor_renders_state_controls_and_full_domain_at_minimum_size() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), backend.name());
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        let test_backend = TestBackend::new(MIN_WIDTH, MIN_HEIGHT);
        let mut terminal = Terminal::new(test_backend).unwrap();

        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains("FAN CURVE"));
        assert!(rendered.contains("AVAILABLE"));
        assert!(rendered.contains("ENTER START"));
        assert!(rendered.contains("MINIMUM"));
        assert!(rendered.contains("PREVIEW"));
        assert!(!rendered.contains("CURRENT"));
        assert!(rendered.contains("100"));
        assert!(rendered.contains("? HELP  Q QUIT"));
        assert!(!rendered.contains("TAB VIEW  P PROFILES"));
    }

    #[test]
    fn host_duty_label_is_preview_draft_or_applied_target() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        let render = |app: &App| {
            let mut terminal = Terminal::new(TestBackend::new(120, 38)).unwrap();
            terminal.draw(|frame| draw(frame, app)).unwrap();
            terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect::<String>()
        };
        assert!(render(&app).contains("PREVIEW"));
        let policy = app.host_editor.policy();
        app.host_start_succeeded(policy);
        assert!(render(&app).contains("TARGET"));
        app.handle_key_event(crossterm::event::KeyCode::Char('s').into());
        assert!(render(&app).contains("DRAFT"));
        assert!(!render(&app).contains("TARGET"));
    }

    #[test]
    fn host_apply_scope_shows_actual_group_id_source_options_at_both_sizes() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        app.handle_key_event(crossterm::event::KeyCode::Enter.into());
        let scope = app.host_apply_scope.as_ref().unwrap();
        let name = scope.selected.name.clone();
        let id = scope.selected.channel_id.0.clone();
        for (width, height) in [(88, 30), (120, 38)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let rendered = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect::<String>();
            assert!(
                rendered.contains(&format!("Only {name} ({id})")),
                "{rendered}"
            );
            assert!(rendered.contains("All fan groups (2)"), "{rendered}");
            assert!(rendered.contains("source CPU"), "{rendered}");
            assert!(rendered.contains("minimum duty"), "{rendered}");
        }
    }

    #[test]
    fn minimum_size_running_host_target_and_presets_remain_visible() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        let mut terminal = Terminal::new(TestBackend::new(MIN_WIDTH, MIN_HEIGHT)).unwrap();
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered: String = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect();
        assert!(rendered.contains("TARGET"));
        assert!(rendered.contains("ENTER SCOPE · O MENU"), "{rendered}");
        assert!(rendered.contains("S src"), "{rendered}");
        assert!(!rendered.contains("S STOP"));
        assert!(!rendered.contains("X src"));
        assert!(rendered.contains("P profiles"));
        assert!(rendered.contains("P PICK"));
        assert!(rendered.contains("PROFILE  Default   P PICK"));
        assert!(!rendered.contains("Silent"));
    }

    #[test]
    fn controller_fault_stays_visible_after_navigation_at_minimum_size() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.host_control.state = HostControlState::RestoreRequired;
        snapshot.host_control.last_error = Some("fan3 RPM below safety threshold".into());
        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let mut app = App::new(snapshot.clone(), "LIVE-TEST");
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            for key in [
                crossterm::event::KeyCode::BackTab,
                crossterm::event::KeyCode::Tab,
                crossterm::event::KeyCode::BackTab,
                crossterm::event::KeyCode::Char('p'),
            ] {
                app.handle_key_event(key.into());
                terminal.draw(|frame| draw(frame, &app)).unwrap();
                let rendered: String = terminal
                    .backend()
                    .buffer()
                    .content()
                    .iter()
                    .map(|cell| cell.symbol())
                    .collect();
                assert!(rendered.contains("Last fan error: fan3 RPM below safety threshold"));
                if app.modal == Some(Modal::HostProfiles) {
                    assert!(rendered.contains("MOTHERBOARD PROFILES"));
                } else if app.editor_mode == EditorMode::Host {
                    assert!(
                        rendered.contains("PREVIEW"),
                        "fault row hid the curve readout"
                    );
                    assert!(rendered.contains("P PICK"), "fault row hid presets");
                }
            }
            snapshot.host_control.last_error = None;
            app.update_telemetry(snapshot.clone());
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let rendered: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(!rendered.contains("Last fan error:"));
            snapshot.host_control.last_error = Some("fan3 RPM below safety threshold".into());
        }
    }

    #[test]
    fn motherboard_presets_replace_architecture_copy_and_picker_only_offers_load() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let rendered: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            let current = app
                .active_host_profile_name()
                .unwrap_or_else(|| "Custom".into());
            assert!(rendered.contains(&format!("PROFILE  {}   P PICK", current)));
            for preset in ["Silent", "Progressive", "Performance"] {
                if current != preset {
                    assert!(
                        !rendered.contains(preset),
                        "unexpected {preset} outside picker"
                    );
                }
            }
            assert!(!rendered.contains("SERVICE MANAGED"));
            assert!(!rendered.contains("saved policy resumes"));
            assert!(
                app.handle_key_event(crossterm::event::KeyCode::Char('p').into())
                    .is_none()
            );
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let picker: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(picker.contains("MOTHERBOARD PROFILES"));
            assert!(picker.contains("22–100 °C"));
            assert!(picker.contains("Silent"));
            assert!(picker.contains("Progressive"));
            assert!(picker.contains("Performance"));
            assert!(!picker.contains("use + apply"));
            assert!(
                app.handle_key_event(crossterm::event::KeyCode::Enter.into())
                    .is_none()
            );
            assert_eq!(app.modal, None);
            assert_eq!(app.host_control_state, HostControlState::Available);
        }
        app.host_start_succeeded(app.host_editor.policy());
        assert_eq!(app.status.text, "Fan control started");
    }

    #[test]
    fn disabled_airflow_is_selectable_and_does_not_advertise_start() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot
            .devices
            .retain(|device| device.id.0 == "kraken-2023" || device.cooling_channels.is_empty());
        snapshot.host_control = Default::default();
        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let mut app = App::new(snapshot.clone(), "DEMO");
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let overview: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(overview.contains("Board: DISABLED"));
            assert!(
                app.handle_key_event(crossterm::event::KeyCode::Tab.into())
                    .is_none()
            );
            assert_eq!(app.editor_mode, EditorMode::Host);
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let rendered: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(rendered.contains("MOTHERBOARD FAN CONTROL"));
            assert!(rendered.contains("Motherboard control is disabled"));
            assert!(rendered.contains("No fan channels are configured"));
            assert!(rendered.contains("CONTROL DISABLED"));
            assert!(rendered.contains("P profiles"));
            assert!(!rendered.contains("ENTER START"));
            assert!(
                app.handle_key_event(crossterm::event::KeyCode::Enter.into())
                    .is_none()
            );
        }
    }

    #[test]
    fn motherboard_status_does_not_displace_the_last_airflow_reading() {
        let mut backend = DemoBackend::new();
        let app = App::new(backend.refresh().unwrap(), "DEMO");
        for width in [120, 180] {
            let area = Rect::new(0, 0, width, 12);
            let mut terminal = Terminal::new(TestBackend::new(width, 12)).unwrap();
            terminal
                .draw(|frame| render_device_overview(frame, area, &app, Theme::default()))
                .unwrap();
            let rendered: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(rendered.contains("Board: AVAILABLE"));
            assert!(
                rendered.contains("IT8689 FAN 4"),
                "last fan label was hidden at width {width}"
            );
            assert!(
                rendered.contains("780"),
                "last fan RPM was hidden at width {width}"
            );
        }
    }

    #[test]
    fn host_view_focuses_airflow_instead_of_the_previous_firmware_device() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.host_control = Default::default();
        let mut app = App::new(snapshot, "DEMO");
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        let area = Rect::new(0, 0, 120, 12);
        let columns = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(36),
                Constraint::Percentage(34),
                Constraint::Percentage(30),
            ])
            .spacing(1)
            .split(area);
        let theme = Theme::default();
        let mut terminal = Terminal::new(TestBackend::new(area.width, area.height)).unwrap();
        terminal
            .draw(|frame| render_device_overview(frame, area, &app, theme))
            .unwrap();
        let buffer = terminal.backend().buffer();
        assert_eq!(buffer[(columns[0].x, 1)].style().fg, theme.border.fg);
        assert_eq!(
            buffer[(columns[1].x, 1)].style().fg,
            theme.focused_border.fg
        );
    }

    #[test]
    fn host_editor_renders_unknown_status_distinct_from_service_restore_required() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), backend.name());
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        app.mark_host_status_unknown();
        let mut terminal = Terminal::new(TestBackend::new(120, 38)).unwrap();
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();
        assert!(rendered.contains("STATUS UNKNOWN"));
        assert!(rendered.contains("O SETTINGS"), "{rendered}");
        assert!(!rendered.contains("RESTORE REQUIRED"));
    }

    #[test]
    fn help_mentions_host_mode_and_source_controls() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), backend.name());
        app.handle_key_event(crossterm::event::KeyCode::Char('?').into());
        let test_backend = TestBackend::new(120, 38);
        let mut terminal = Terminal::new(test_backend).unwrap();

        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains("Cycle firmware devices / motherboard fans"));
        assert!(rendered.contains("Pick: host loads draft; firmware applies"));
        assert!(!rendered.contains("Toggle firmware / host policy editor"));
        assert!(rendered.contains("S                Host: cycle source CPU/GPU/MAX"));
        assert!(rendered.contains("O                Settings: app preference"));
        assert!(rendered.contains("Opt in to monitoring / apply changes"));
        assert!(rendered.contains("Hotter follow raises; cooler follow reductions"));
    }

    #[test]
    fn minimum_size_help_keeps_both_editors_neighbor_hint_visible() {
        for mode in [EditorMode::Firmware, EditorMode::Host] {
            let mut backend = DemoBackend::new();
            let mut app = App::new(backend.refresh().unwrap(), backend.name());
            app.editor_mode = mode;
            app.handle_key_event(crossterm::event::KeyCode::Char('?').into());
            let mut terminal = Terminal::new(TestBackend::new(MIN_WIDTH, MIN_HEIGHT)).unwrap();
            terminal.draw(|frame| draw(frame, &app)).unwrap();
            let rendered: String = terminal
                .backend()
                .buffer()
                .content()
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(rendered.contains("Curve ↑ / ↓"));
            assert!(rendered.contains("Hotter follow raises; cooler follow reductions"));
            assert!(rendered.contains("Quit (fan control continues)"));
        }
    }

    #[test]
    fn live_unverified_curve_and_apply_confirmation_are_visible() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].cooling_channels[0].curve_state = crate::model::CurveState::Unverified;
        snapshot.monitoring.opted_in = true;
        let mut app = App::with_runtime_config(
            snapshot,
            "LIQUIDCTL",
            Vec::new(),
            crate::config::AppConfig::default(),
        );
        let test_backend = TestBackend::new(120, 38);
        let mut terminal = Terminal::new(test_backend).unwrap();

        app.handle_key_event(crossterm::event::KeyCode::Enter.into());
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains("UNVERIFIED"));
        assert!(rendered.contains("CONFIRM FIRMWARE CURVE WRITE"));
        assert!(rendered.contains("Y apply curve"));
        assert!(rendered.contains("N / Esc cancel"));
    }

    #[test]
    fn firmware_and_host_rename_modals_render_at_both_terminal_sizes() {
        use crate::profile::{ProfileCurve, ProfileLibrary, SavedProfile};
        for (width, height) in [(MIN_WIDTH, MIN_HEIGHT), (120, 38)] {
            let mut backend = DemoBackend::new();
            let snapshot = backend.refresh().unwrap();
            let mut library = ProfileLibrary::default();
            library.profiles.push(SavedProfile {
                id: 1,
                name: "Work profile".into(),
                curve: ProfileCurve::Firmware(
                    crate::profile::built_in_profiles()[0].points.clone(),
                ),
            });
            library.profiles.push(SavedProfile {
                id: 2,
                name: "my-custom-profile-123456789".into(),
                curve: ProfileCurve::Host(
                    crate::profile::built_in_host_profiles(
                        crate::model::HostTemperatureSource::Gpu,
                        30,
                    )[0]
                    .curve
                    .clone(),
                ),
            });
            library.next_index = 3;
            let mut app = App::with_profile_library(
                snapshot,
                "DEMO",
                library,
                crate::config::AppConfig::default(),
            );
            app.handle_key_event(crossterm::event::KeyCode::Char('p').into());
            let picker = dashboard_rows(&app, width, height).join("\n");
            assert!(picker.contains("COOLING PROFILE LIBRARY"));
            assert!(picker.contains("Work profile"));
            app.handle_key_event(crossterm::event::KeyCode::End.into());
            app.handle_key_event(crossterm::event::KeyCode::Char('r').into());
            assert!(
                dashboard_rows(&app, width, height)
                    .join("\n")
                    .contains("RENAME CUSTOM PROFILE")
            );
            app.handle_key_event(crossterm::event::KeyCode::Esc.into());
            app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
            app.handle_key_event(crossterm::event::KeyCode::Char('p').into());
            app.handle_key_event(crossterm::event::KeyCode::End.into());
            let picker = dashboard_rows(&app, width, height).join("\n");
            assert!(picker.contains("my-custom-profile-123456789"), "{picker}");
            app.handle_key_event(crossterm::event::KeyCode::Char('r').into());
            assert!(
                dashboard_rows(&app, width, height)
                    .join("\n")
                    .contains("RENAME CUSTOM PROFILE")
            );
        }
    }

    #[test]
    fn minimum_width_keeps_offline_state_visible() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].online = false;
        let app = App::new(snapshot, backend.name());
        let test_backend = TestBackend::new(MIN_WIDTH, MIN_HEIGHT);
        let mut terminal = Terminal::new(test_backend).unwrap();

        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains('×'));
        assert!(rendered.contains("PROFILE"));
        assert!(rendered.contains("P PICK"));
    }

    #[test]
    fn it8689_rpm_and_mode_rows_keep_spaces_at_narrow_and_wide_widths() {
        use crate::model::Reading;

        let device = Device {
            id: DeviceId::new("host-telemetry"),
            name: "Host telemetry".into(),
            model: "Read-only host sensors".into(),
            kind: DeviceKind::FanController,
            online: true,
            readings: vec![
                Reading::new("IT8689 fan 3", 1446.0, "rpm", ReadingKind::Speed),
                Reading::new("IT8689 fan 3 mode", 1.0, "", ReadingKind::Mode),
                Reading::new("IT8689 fan 4", 1171.0, "rpm", ReadingKind::Speed),
                Reading::new("IT8689 fan 4 mode", 0.0, "", ReadingKind::Mode),
            ],
            cooling_channels: Vec::new(),
        };
        for width in [25, 30, 40] {
            let lines = device_summary_lines(&device, None, Theme::default(), width);
            assert!(lines.iter().all(|line| line.width() <= usize::from(width)));
            let rows = lines
                .into_iter()
                .map(|line| {
                    line.spans
                        .into_iter()
                        .map(|span| span.content.into_owned())
                        .collect::<String>()
                })
                .collect::<Vec<_>>();
            let speed = &rows[1];
            let manual = &rows[2];
            let full_speed = &rows[4];
            assert!(speed.contains("FAN 3  1446"), "{width}: {speed}");
            assert!(manual.contains("FAN 3"), "{width}: {manual}");
            assert!(manual.contains("MANUAL"), "{width}: {manual}");
            assert!(!manual.contains("(1)"), "{width}: {manual}");
            assert!(!manual.contains("MODEMANUAL"), "{width}: {manual}");
            assert!(full_speed.contains("FAN 4"), "{width}: {full_speed}");
            assert!(full_speed.contains("FULL SPEED"), "{width}: {full_speed}");
            assert!(!full_speed.contains("(0)"), "{width}: {full_speed}");
            assert!(!full_speed.contains("MODEFULL"), "{width}: {full_speed}");
        }
    }

    #[test]
    fn mode_readings_render_their_conservative_text() {
        let device = Device {
            id: DeviceId::new("host-telemetry"),
            name: "Host telemetry".into(),
            model: "Read-only host sensors".into(),
            kind: DeviceKind::FanController,
            online: true,
            readings: vec![crate::model::Reading::new(
                "IT8689 fan 3 mode",
                2.0,
                "",
                ReadingKind::Mode,
            )],
            cooling_channels: Vec::new(),
        };

        let rendered = device_summary_lines(&device, None, Theme::default(), 40)
            .into_iter()
            .flat_map(|line| line.spans)
            .map(|span| span.content.into_owned())
            .collect::<String>();

        assert!(rendered.contains("IT8689 FAN 3 MODE"));
        assert!(rendered.contains("AUTO"));
        assert!(!rendered.contains("(2)"));
    }

    #[test]
    fn small_terminal_shows_a_clear_fallback() {
        let mut backend = DemoBackend::new();
        let app = App::new(backend.refresh().unwrap(), backend.name());
        let test_backend = TestBackend::new(60, 20);
        let mut terminal = Terminal::new(test_backend).unwrap();

        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains("Terminal too small"));
    }
}
