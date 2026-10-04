use ratatui::style::{Color, Modifier, Style};

#[derive(Clone, Copy, Debug)]
pub struct Theme {
    pub text: Style,
    pub strong: Style,
    pub muted: Style,
    pub border: Style,
    pub focused_border: Style,
    pub bar: Style,
    pub selected_bar: Style,
    pub selected: Style,
    pub grid: Style,
    pub error: Style,
}

impl Theme {
    pub fn monochrome() -> Self {
        Self {
            text: Style::default().fg(Color::Gray),
            strong: Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
            muted: Style::default().fg(Color::DarkGray),
            border: Style::default().fg(Color::DarkGray),
            focused_border: Style::default().fg(Color::White),
            bar: Style::default().fg(Color::Gray),
            selected_bar: Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
            selected: Style::default()
                .fg(Color::Black)
                .bg(Color::White)
                .add_modifier(Modifier::BOLD),
            grid: Style::default().fg(Color::DarkGray),
            error: Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD | Modifier::REVERSED),
        }
    }
}

impl Default for Theme {
    fn default() -> Self {
        Self::monochrome()
    }
}
