use ratatui::{buffer::Buffer, layout::Rect, style::Style, widgets::Widget};

use crate::{
    model::{CurvePoint, HostCurvePoint},
    theme::Theme,
};

pub trait ChartPoint {
    fn temperature_label(&self) -> String;
    fn duty(&self) -> u8;
}

impl ChartPoint for CurvePoint {
    fn temperature_label(&self) -> String {
        self.temperature.to_string()
    }

    fn duty(&self) -> u8 {
        self.duty
    }
}

impl ChartPoint for HostCurvePoint {
    fn temperature_label(&self) -> String {
        let whole = self.temperature_millidegrees / 1_000;
        let remainder = self.temperature_millidegrees.abs() % 1_000;
        if remainder == 0 {
            whole.to_string()
        } else {
            format!("{whole}.{remainder:03}")
        }
    }

    fn duty(&self) -> u8 {
        self.duty_percent
    }
}

#[derive(Clone, Copy)]
pub struct CurveChart<'a, P> {
    points: &'a [P],
    selected: usize,
    theme: Theme,
}

impl<'a, P> CurveChart<'a, P> {
    pub fn new(points: &'a [P], selected: usize, theme: Theme) -> Self {
        Self {
            points,
            selected,
            theme,
        }
    }
}

impl<P: ChartPoint> Widget for CurveChart<'_, P> {
    fn render(self, area: Rect, buffer: &mut Buffer) {
        if area.width < 46 || area.height < 7 || self.points.is_empty() {
            buffer.set_string(
                area.x,
                area.y,
                "Curve chart needs more terminal space",
                self.theme.muted,
            );
            return;
        }

        let axis_width = 5;
        let label_height = 2;
        let plot = Rect::new(
            area.x + axis_width,
            area.y,
            area.width - axis_width,
            area.height - label_height,
        );
        let bottom = plot.y + plot.height - 1;

        for percentage in [100_u16, 75, 50, 25, 0] {
            let y = plot.y + ((100 - percentage) * plot.height.saturating_sub(1) / 100);
            buffer.set_string(area.x, y, format!("{percentage:>3}┤"), self.theme.muted);
            for x in plot.x..plot.right() {
                buffer[(x, y)].set_symbol("·").set_style(self.theme.grid);
            }
        }

        // Sample only the *rendered bars*, not the authoritative curve. Reserve
        // a column between bars even for a 64-point policy on a narrow chart.
        let visible_count = self.points.len().min(usize::from(plot.width.div_ceil(2)));
        let indices = sampled_indices(self.points.len(), visible_count, self.selected);
        let point_count = visible_count as u16;
        let (offset, bar_width, gap) = bar_geometry(plot.width, point_count);
        let mut labels = Vec::new();
        for (bar, &original_index) in indices.iter().enumerate() {
            let point = &self.points[original_index];
            let index = bar as u16;
            let start = plot.x + offset + index * (bar_width + gap);
            let end = start + bar_width;
            let center = start + (bar_width - 1) / 2;
            let selected = original_index == self.selected;
            let style = if selected {
                self.theme.selected_bar
            } else {
                self.theme.bar
            };

            if selected {
                for y in plot.y..=bottom {
                    let cell = &mut buffer[(center, y)];
                    if cell.symbol() == " " || cell.symbol() == "·" {
                        cell.set_symbol("┊").set_style(self.theme.grid);
                    }
                }
            }

            let (full_rows, partial_eighths) = fill_geometry(point.duty(), plot.height);
            let top = bottom.saturating_add(1).saturating_sub(full_rows);
            for x in start..end {
                for y in top..=bottom {
                    buffer[(x, y)].set_symbol("█").set_style(style);
                }
                if partial_eighths > 0 {
                    let partial_y = top.saturating_sub(1).max(plot.y);
                    buffer[(x, partial_y)]
                        .set_symbol(partial_block(partial_eighths))
                        .set_style(style);
                }
            }

            let endpoint = index == 0 || index + 1 == point_count;
            let tick = index.is_multiple_of(5) && visible_count == self.points.len();
            if endpoint || tick || selected {
                let marker_style = if selected {
                    self.theme.strong
                } else {
                    self.theme.muted
                };
                buffer[(center, plot.bottom())]
                    .set_symbol(if selected { "▲" } else { "┬" })
                    .set_style(marker_style);
                // Endpoints take precedence over the selected label, which in
                // turn takes precedence over incidental fifth-point labels.
                labels.push((
                    if endpoint {
                        0
                    } else if selected {
                        1
                    } else {
                        2
                    },
                    center,
                    point.temperature_label(),
                    marker_style,
                ));
            }
        }

        labels.sort_by_key(|label| label.0);
        let mut occupied = Vec::new();
        for (priority, center, label, style) in labels {
            let width = u16::try_from(label.len())
                .unwrap_or(u16::MAX)
                .min(plot.width);
            let preferred = center
                .saturating_sub(width / 2)
                .clamp(plot.x, plot.right() - width);
            let free = |x: u16| {
                let end = x + width;
                !occupied
                    .iter()
                    .any(|&(start, stop): &(u16, u16)| x <= stop && start <= end)
            };
            // Keep endpoints fixed. If the selected bar sits next to one,
            // move only its *label*, not the bar or its position marker.
            let position = if free(preferred) {
                Some(preferred)
            } else if priority == 1 {
                (1..plot.width).find_map(|distance| {
                    [
                        preferred.checked_add(distance),
                        preferred.checked_sub(distance),
                    ]
                    .into_iter()
                    .flatten()
                    .find(|&x| {
                        x >= plot.x
                            && x.checked_add(width).is_some_and(|end| end <= plot.right())
                            && free(x)
                    })
                })
            } else {
                None
            };
            if let Some(x) = position {
                write_clipped(buffer, x, plot.bottom() + 1, plot.right(), &label, style);
                occupied.push((x, x + width));
            }
        }
    }
}

// Evenly sample the display, then exchange the nearest interior sample for
// the edited point. Replacing (rather than inserting) keeps spacing and order.
fn sampled_indices(point_count: usize, visible_count: usize, selected: usize) -> Vec<usize> {
    let mut indices = (0..visible_count)
        .map(|bar| {
            if visible_count == point_count || visible_count == 1 {
                bar
            } else {
                bar * (point_count - 1) / (visible_count - 1)
            }
        })
        .collect::<Vec<_>>();

    if selected < point_count && visible_count >= 3 && !indices.contains(&selected) {
        let next = indices.partition_point(|&index| index < selected);
        let previous = next - 1;
        let replace = if previous == 0 {
            next
        } else if next == visible_count - 1
            || selected - indices[previous] <= indices[next] - selected
        {
            previous
        } else {
            next
        };
        indices[replace] = selected;
    }
    indices
}

fn fill_geometry(duty: u8, plot_height: u16) -> (u16, u8) {
    let available_eighths = plot_height * 8;
    let filled_eighths = (u16::from(duty.min(100)) * available_eighths + 50) / 100;
    (filled_eighths / 8, (filled_eighths % 8) as u8)
}

fn partial_block(eighths: u8) -> &'static str {
    match eighths {
        1 => "▁",
        2 => "▂",
        3 => "▃",
        4 => "▄",
        5 => "▅",
        6 => "▆",
        7 => "▇",
        _ => " ",
    }
}

fn bar_geometry(plot_width: u16, point_count: u16) -> (u16, u16, u16) {
    let gap = 1;
    let total_gap_width = gap * point_count.saturating_sub(1);
    let bar_width = (plot_width - total_gap_width) / point_count;
    let content_width = bar_width * point_count + total_gap_width;
    let offset = (plot_width - content_width) / 2;
    (offset, bar_width, gap)
}

fn write_clipped(buffer: &mut Buffer, x: u16, y: u16, right: u16, text: &str, style: Style) {
    if x < right {
        buffer.set_stringn(x, y, text, usize::from(right - x), style);
    }
}

#[cfg(test)]
mod tests {
    use ratatui::{buffer::Buffer, layout::Rect, widgets::Widget};

    use super::*;

    #[test]
    fn adjacent_percentages_have_distinct_heights_when_plot_is_tall_enough() {
        let geometries = (0..=100)
            .map(|duty| fill_geometry(duty, 13))
            .collect::<Vec<_>>();

        assert!(geometries.windows(2).all(|window| window[0] != window[1]));
    }

    #[test]
    fn fractional_height_uses_lower_block_glyphs() {
        assert_eq!(fill_geometry(50, 13), (6, 4));
        assert_eq!(partial_block(1), "▁");
        assert_eq!(partial_block(4), "▄");
        assert_eq!(partial_block(7), "▇");
    }

    #[test]
    fn bar_geometry_keeps_every_bar_the_same_width() {
        assert_eq!(bar_geometry(53, 27), (0, 1, 1));
        assert_eq!(bar_geometry(70, 35), (0, 1, 1));
        assert_eq!(bar_geometry(85, 40), (3, 1, 1));
        assert_eq!(bar_geometry(120, 40), (0, 2, 1));
    }

    #[test]
    fn forty_points_keep_gaps_and_endpoints_at_narrow_and_wide_plot_widths() {
        let points = (22..62)
            .map(|temperature| CurvePoint {
                temperature,
                duty: 50,
            })
            .collect::<Vec<_>>();

        for (plot_width, expected_count, selected) in [(53, 27, 11), (70, 35, 7), (85, 40, 11)] {
            let area = Rect::new(0, 0, plot_width + 5, 12);
            let mut buffer = Buffer::empty(area);
            if expected_count < points.len() {
                assert!(
                    !sampled_indices(points.len(), expected_count, points.len())
                        .contains(&selected)
                );
            }
            let indices = sampled_indices(points.len(), expected_count, selected);
            assert_eq!(indices.len(), expected_count);
            assert_eq!(indices[0], 0);
            assert_eq!(indices.last(), Some(&39));
            assert!(indices.windows(2).all(|pair| pair[0] < pair[1]));
            assert!(indices.contains(&selected));
            assert_eq!(points.len(), 40); // Display sampling never edits the curve.

            CurveChart::new(&points, selected, Theme::monochrome()).render(area, &mut buffer);
            let (offset, bar_width, gap) = bar_geometry(plot_width, expected_count as u16);
            let bar_y = 9; // All 50% bars cover the chart's bottom plot row.
            let marker_y = 10;
            let bar_x = |bar: usize| 5 + offset + bar as u16 * (bar_width + gap);
            for bar in 0..expected_count {
                let x = bar_x(bar);
                assert_eq!(buffer[(x, bar_y)].symbol(), "█");
                if bar + 1 < expected_count {
                    assert_ne!(buffer[(x + bar_width, bar_y)].symbol(), "█");
                }
            }
            let selected_bar = indices.iter().position(|&index| index == selected).unwrap();
            assert_eq!(buffer[(bar_x(selected_bar), marker_y)].symbol(), "▲");
            assert_eq!(
                (5..area.width)
                    .filter(|&x| buffer[(x, marker_y)].symbol() == "▲")
                    .count(),
                1
            );
            let label_row = (5..area.width)
                .map(|x| buffer[(x, 11)].symbol())
                .collect::<String>();
            assert!(label_row.contains("22"), "width {plot_width}: {label_row}");
            let selected_label = points[selected].temperature.to_string();
            assert!(
                label_row.contains(&selected_label),
                "width {plot_width}: {label_row}"
            );
            assert!(label_row.contains("61"), "width {plot_width}: {label_row}");
        }
    }

    #[test]
    fn selected_temperature_label_remains_visible_next_to_endpoints() {
        let points = (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: 50,
            })
            .collect::<Vec<_>>();
        for plot_width in [53, 85] {
            for (selected, selected_label) in [(1, "21"), (38, "58")] {
                let area = Rect::new(0, 0, plot_width + 5, 12);
                let mut buffer = Buffer::empty(area);
                CurveChart::new(&points, selected, Theme::monochrome()).render(area, &mut buffer);
                let label_row = (5..area.width)
                    .map(|x| buffer[(x, area.bottom() - 1)].symbol())
                    .collect::<String>();
                assert!(label_row.contains("20"), "{plot_width}: {label_row}");
                assert!(label_row.contains("59"), "{plot_width}: {label_row}");
                assert!(
                    label_row.contains(selected_label),
                    "{plot_width}: {label_row}"
                );
            }
        }
    }

    #[test]
    fn unsampled_selected_points_near_both_endpoints_remain_visible() {
        let points = (0..64)
            .map(|temperature| CurvePoint {
                temperature,
                duty: 50,
            })
            .collect::<Vec<_>>();
        let area = Rect::new(0, 0, 58, 12); // 53 plot columns: 27 separated bars.
        let ordinary = sampled_indices(64, 27, 64);

        for selected in [1, 30, 62] {
            assert!(!ordinary.contains(&selected));
            let indices = sampled_indices(64, 27, selected);
            assert_eq!(indices[0], 0);
            assert_eq!(indices.last(), Some(&63));
            assert!(indices.windows(2).all(|pair| pair[0] < pair[1]));
            assert!(indices.contains(&selected));
            assert_eq!(indices.len(), 27);
            assert_eq!(points.len(), 64);

            let mut buffer = Buffer::empty(area);
            CurveChart::new(&points, selected, Theme::monochrome()).render(area, &mut buffer);
            let bar = indices.iter().position(|&index| index == selected).unwrap();
            let x = 5 + bar as u16 * 2;
            assert_eq!(buffer[(x, 10)].symbol(), "▲");
            assert_eq!(buffer[(x, 9)].symbol(), "█");
            assert_ne!(buffer[(x + 1, 9)].symbol(), "█");
            let label_row = (5..area.width)
                .map(|x| buffer[(x, 11)].symbol())
                .collect::<String>();
            assert!(label_row.contains('0'));
            assert!(label_row.contains("63"));
            if selected == 30 {
                assert!(label_row.contains("30"));
            }
        }
    }

    #[test]
    fn sampling_includes_any_selected_point_without_duplicating_or_reordering_bars() {
        for (point_count, visible_count) in [(40, 27), (40, 35), (40, 40), (64, 27)] {
            for selected in 0..point_count {
                let indices = sampled_indices(point_count, visible_count, selected);
                assert_eq!(indices.len(), visible_count);
                assert_eq!(indices[0], 0);
                assert_eq!(indices.last(), Some(&(point_count - 1)));
                assert!(indices.contains(&selected));
                assert!(indices.windows(2).all(|pair| pair[0] < pair[1]));
            }
        }
    }

    #[test]
    fn endpoint_labels_stay_inside_an_offset_chart() {
        let points = (0..40)
            .map(|index| HostCurvePoint {
                temperature_millidegrees: 22_000 + index * 2_000,
                duty_percent: 50,
            })
            .collect::<Vec<_>>();
        let area = Rect::new(4, 2, 58, 12);
        let mut buffer = Buffer::empty(Rect::new(0, 0, 70, 18));

        CurveChart::new(&points, 39, Theme::monochrome()).render(area, &mut buffer);
        let label_row = area.bottom() - 1;
        assert_eq!(buffer[(59, label_row)].symbol(), "1");
        assert_eq!(buffer[(60, label_row)].symbol(), "0");
        assert_eq!(buffer[(61, label_row)].symbol(), "0");
        assert_eq!(buffer[(62, label_row)].symbol(), " ");
        assert_eq!(buffer[(61, area.bottom() - 2)].symbol(), "▲");
    }

    #[test]
    fn host_points_render_their_full_22_to_100_degree_domain() {
        let points = (0..40)
            .map(|index| HostCurvePoint {
                temperature_millidegrees: 22_000 + index * 2_000,
                duty_percent: if index == 39 { 100 } else { 50 },
            })
            .collect::<Vec<_>>();
        let area = Rect::new(0, 0, 70, 12);
        let mut buffer = Buffer::empty(area);

        CurveChart::new(&points, 39, Theme::monochrome()).render(area, &mut buffer);
        let rendered = buffer
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains("22"));
        assert!(rendered.contains("100"));
        assert!(rendered.contains('▲'));
    }

    #[test]
    fn sixty_four_point_running_policy_renders_at_minimum_chart_width() {
        let points = (0..64)
            .map(|index| HostCurvePoint {
                temperature_millidegrees: 22_000 + index * 1_000,
                duty_percent: if index == 63 { 100 } else { 50 },
            })
            .collect::<Vec<_>>();
        let area = Rect::new(0, 0, 58, 12);
        let mut buffer = Buffer::empty(area);
        CurveChart::new(&points, 63, Theme::monochrome()).render(area, &mut buffer);
        let rendered = buffer
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();
        assert!(!rendered.contains("more horizontal space"));
        assert!(rendered.contains("22"));
        assert!(rendered.contains("85"));
        assert!(rendered.contains('▲'));
    }

    #[test]
    fn out_of_range_duties_do_not_draw_above_the_chart() {
        let points = (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: 250,
            })
            .collect::<Vec<_>>();
        let buffer_area = Rect::new(0, 0, 80, 24);
        let chart_area = Rect::new(5, 5, 65, 12);
        let mut buffer = Buffer::empty(buffer_area);

        CurveChart::new(&points, 0, Theme::monochrome()).render(chart_area, &mut buffer);

        for y in 0..chart_area.y {
            for x in 0..buffer_area.width {
                assert_ne!(buffer[(x, y)].symbol(), "█");
            }
        }
    }

    #[test]
    fn chart_renders_all_points_and_selection_marker() {
        let points = (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: 50,
            })
            .collect::<Vec<_>>();
        let area = Rect::new(0, 0, 65, 12);
        let mut buffer = Buffer::empty(area);

        CurveChart::new(&points, 11, Theme::monochrome()).render(area, &mut buffer);
        let rendered = buffer
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();

        assert!(rendered.contains('█'));
        assert!(rendered.contains('▲'));
        assert!(rendered.contains("100"));
        assert!(rendered.contains("20"));
        assert!(rendered.contains("59"));
    }
}
