use clap::Parser;
use nzxt_cam_tui::{backend::DemoBackend, ipc::IpcBackend, run};

#[derive(Debug, Parser)]
#[command(version, about)]
struct Cli {
    /// Run with simulated devices and without the hardware service.
    #[arg(long)]
    demo: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    if cli.demo {
        ratatui::run(|terminal| run(terminal, DemoBackend::new()))?;
    } else {
        // Connect before ratatui changes terminal state, so startup failures are
        // printed normally and never leave a partially initialized terminal.
        let backend = IpcBackend::new()?;
        ratatui::run(|terminal| run(terminal, backend))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::*;

    #[test]
    fn live_mode_is_the_default() {
        let cli = Cli::try_parse_from(["nzxt-cam-tui"]).unwrap();
        assert!(!cli.demo);
    }

    #[test]
    fn demo_is_parseable_and_service_free() {
        let cli = Cli::try_parse_from(["nzxt-cam-tui", "--demo"]).unwrap();
        assert!(cli.demo);
    }

    #[test]
    fn removed_liquidctl_path_option_is_rejected() {
        let result = Cli::try_parse_from([
            "nzxt-cam-tui",
            "--liquidctl-path",
            "/opt/liquidctl/bin/liquidctl",
        ]);
        assert!(result.is_err());
    }
}
