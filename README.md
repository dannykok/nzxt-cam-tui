# nzxt-cam-tui

A keyboard-first Linux hardware monitor and cooling-curve editor for NZXT and liquidctl-compatible devices. The TUI talks to the `nzxt-cam-hwd` service over a local Unix socket; it does not access hardware directly.

## Features

- AIO, fan, USB and optional host telemetry, grouped in a compact dashboard.
- Editable 40-point pump/fan firmware curves for supported Kraken Z3/X3 devices, plus built-in and saved custom profiles. Edits do not write hardware until applied; later firmware applies require confirmation by default.
- Kraken 2023 LCD face picker with CPU, GPU, liquid and combined temperature presets. The service maintains the selected face after the TUI exits.
- Motherboard fan-policy view with CPU/GPU/MAX temperature sources and per-group curves. Motherboard control is **disabled by default**; the packaged example keeps its channel configuration commented out because safe startup and minimum duties have not been established. Do not enable it without a separate safety assessment.

During service startup, the TUI connects but hardware requests remain unavailable until recovery and saved-setting replay finish. Before the service has a saved opt-in, the overview is read-only. **Enter** opts in: it can write available AIO curves and start any available motherboard fan control, with separate results for each component. **O** opens Settings for future auto-resume and immediate motherboard Stop. Quitting the TUI does **not** stop service-owned control or LCD updates. The curve editor needs a terminal of at least 88×30.

To try the interface without a service or hardware, run `cargo run -- --demo` from the repository root.

## Prerequisites

Connecting to hardware requires Linux, systemd, `liquidctl` 1.16+ at `/usr/bin/liquidctl`, and a Unicode/256-color terminal. Building requires Rust 1.88+. Users must belong to `nzxt-cam-control`; the initial setup below creates the group and adds the user.

## Motherboard fan control

Motherboard fan control uses the kernel's `it87` module and is limited to the exact Gigabyte B650 AORUS ELITE AX ICE / IT8689 hardware supported by this project. AIO monitoring/control and the LCD do not need this module. Check whether your **running kernel** provides it:

```bash
modinfo it87
```

If `modinfo` cannot find it, install a package containing `it87` **for your running kernel**, using your distribution's package manager. For example, [some Ubuntu kernels include it in `linux-modules-extra`](https://packages.ubuntu.com/eu/jammy-updates/amd64/linux-modules-extra-6.8.0-138-generic/filelist); on a compatible Ubuntu kernel:

```bash
sudo apt install "linux-modules-extra-$(uname -r)"
modinfo it87
```

The standard [Arch Linux `linux` kernel includes `it87`](https://archlinux.org/packages/core/x86_64/linux/files/); other kernels and distributions may package it differently. A package alone does not load the module. **Before loading it**, verify the board identity and assess physical fan mapping, safe minimum duties, startup and recovery. Loading a hardware driver is not a substitute for that assessment. When safe to proceed, load it and inspect the discovered devices:

```bash
cat /sys/class/dmi/id/board_vendor /sys/class/dmi/id/board_name
sudo modprobe it87
lsmod | grep '^it87'
for hw in /sys/class/hwmon/hwmon*; do
    case "$(cat "$hw/name")" in
        it8689|it8689_*) printf '%s -> %s\n' "$hw" "$(readlink -f "$hw")" ;;
    esac
done
```

The service requires **exactly one** IT8689 device whose resolved path contains `it87.2624`. If that check fails, stop here; do not force a chip ID or bypass ACPI resource checks. After confirming the identity and safe operation, configure systemd to load the module on future boots:

```bash
printf 'it87\n' | sudo tee /etc/modules-load.d/nzxt-cam-it87.conf
```

Reboot and repeat the identity check **before** using motherboard control. The service does not load the driver itself. Keep the `host_control` entries in `packaging/nzxt-cam-hardware.toml.example` commented out until the safety assessment is complete; its example minimum duties have **not** been verified as safe. Do not enable fan control just to clear a discovery warning.

## Installation

**Install the service and TUI from the same build:** they use a fixed v6 protocol, so upgrading only the TUI can leave it unable to connect.

### Existing systemd installation

Build as your regular user, review the plan, then run the interactive installer:

```bash
scripts/build-release.sh
scripts/install.sh --dry-run
sudo scripts/install.sh
```

The release helper builds the matching pair with private source paths remapped; use it for distributed binaries. The installer requires the existing `/usr/bin/nzxt-cam-hwd` and `/usr/bin/nzxt-cam-tui` pair and installed systemd units. It backs up both binaries, checks for unresolved recovery state, installs the matching pair, project license and bundled-font notices, and restarts only units that were already active. It asks before stopping an active service/socket; **Enter at the prompt means yes**. A stop can interrupt fan control and return motherboard fans to BIOS control. An enabled saved policy may resume when the service restarts (or when socket activation starts it on the next TUI connection). The installer preserves saved policy and does not disarm it or enable units. If installation fails, inspect unit and recovery state before any manual restart; a failure after replacement leaves the units stopped. If you need to prevent saved control from resuming, stop the socket and service and use the *old installed service's* explicit `restore` command to disarm intent; verify BIOS ownership and clear recovery state before replacing the binaries. Do not treat a restart or closing the TUI as Stop.

### Initial setup (no existing systemd installation)

Build the same pair as above, then install the binaries, font notices, group declaration and socket/service units:

```bash
sudo install -Dm755 target/release/nzxt-cam-hwd /usr/bin/nzxt-cam-hwd
sudo install -Dm755 target/release/nzxt-cam-tui /usr/bin/nzxt-cam-tui
sudo install -Dm644 LICENSE /usr/share/licenses/nzxt-cam-tui/LICENSE
sudo install -Dm644 crates/hwd/assets/OFL.txt /usr/share/licenses/nzxt-cam-hwd/OFL.txt
sudo install -Dm644 crates/hwd/assets/NOTICE.md /usr/share/licenses/nzxt-cam-hwd/NOTICE.md
sudo install -Dm644 packaging/sysusers.d/nzxt-cam.conf /usr/lib/sysusers.d/nzxt-cam.conf
sudo install -Dm644 packaging/systemd/nzxt-cam-hwd.socket /usr/lib/systemd/system/nzxt-cam-hwd.socket
sudo install -Dm644 packaging/systemd/nzxt-cam-hwd.service /usr/lib/systemd/system/nzxt-cam-hwd.service
sudo systemd-sysusers /usr/lib/sysusers.d/nzxt-cam.conf
sudo usermod -aG nzxt-cam-control "$USER"
sudo systemctl daemon-reload
sudo systemctl enable --now nzxt-cam-hwd.socket
```

Log out and back in for group membership to take effect, then run `nzxt-cam-tui`. The socket activates the service on connection. If you want service startup without a TUI, you can enable the service separately; **starting it can resume a saved fan policy and AIO settings**, so check existing policy and recovery state first. Do not overwrite a running service binary: its post-stop recovery command uses that path. After the first opt-in, saved AIO curves and the selected LCD face may also resume on future service starts when auto-resume is enabled.

## Test

From the repository root (no service or hardware required):

```bash
cargo test --locked --workspace --all-targets
bash scripts/build-release_tests.sh
bash scripts/install_tests.sh
./scripts/verify-systemd.sh
```

The shell tests use fake Cargo/systemd state. The systemd check stages both binaries, licenses, configuration and units in a temporary root; it does not install or start the service.
