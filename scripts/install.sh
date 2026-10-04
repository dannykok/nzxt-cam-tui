#!/usr/bin/env bash
# Upgrade the matching prebuilt binaries in an existing systemd installation.
# Never build as root, run restore, or discard fan-control recovery state.
set -euo pipefail
umask 077
PATH=/usr/sbin:/usr/bin:/sbin:/bin

service=nzxt-cam-hwd.service
socket=nzxt-cam-hwd.socket
install_dir=/usr/bin
license_dir=/usr/share/licenses/nzxt-cam-hwd
project_license_dir=/usr/share/licenses/nzxt-cam-tui
backup_parent=/var/backups/nzxt-cam
runtime=/run/nzxt-cam
policy_dir=/var/lib/nzxt-cam

fail() { printf 'install.sh: %s\n' "$*" >&2; exit 1; }

acceptable_unit_state() { [[ $1 == active || $1 == inactive ]]; }
needs_confirmation() { [[ $1 == active || $2 == active ]]; }
confirmed_choice() { [[ $1 == '' || $1 == [Yy] || $1 == [Yy][Ee][Ss] ]]; }

confirmation_prompt() {
    printf '%s\n' \
        'The NZXT hardware service/socket will STOP for installation.' \
        'Stopping can interrupt active fan control and return fans to BIOS control.' \
        'Proceed?(Y/n)'
}

installation_success_message() {
    printf 'NZXT CAM installed successfully.\n'
    if [[ $1 == active ]]; then
        printf 'Hardware service is running.\n'
    else
        printf 'To start the hardware service when ready:\n  sudo systemctl start nzxt-cam-hwd.service\n'
    fi
}

unit_state() {
    local unit=$1 state
    [[ $(systemctl show -P LoadState "$unit") == loaded ]] || fail "missing systemd unit: $unit"
    state=$(systemctl show -P ActiveState "$unit") || fail "cannot inspect $unit"
    acceptable_unit_state "$state" || fail "$unit is $state; resolve that state before installing"
    printf '%s\n' "$state"
}

# No record, temporary record or cleared marker may be ignored or removed to
# make an upgrade work. The durable enabled policy is deliberately preserved.
recovery_clear() {
    local normal=$1 policy=$2 path
    for path in \
        "$normal/host-control.json" "$normal/host-control.json.tmp" "$normal/host-control.restored" \
        "$policy/active-policy.json.tmp" "$policy/monitor-intent.json.tmp"; do
        if [[ -e $path || -L $path ]]; then
            printf 'install.sh: unresolved recovery/policy state at %s; leave it intact and resolve it first\n' "$path" >&2
            return 1
        fi
    done
}

# Retired units reference removed commands in ExecStopPost. Even a cleanly
# inactive unit must be uninstalled before replacing its binary; never run it.
retired_units_absent() {
    local unit loaded
    for unit in \
        nzxt-cam-it8689-fan3-handoff.service nzxt-cam-it8689-fan4-handoff.service \
        nzxt-cam-it8689-dual-trial.service nzxt-cam-it8689-fan3-probe.service \
        nzxt-cam-it8689-fan4-probe.service; do
        loaded=$(systemctl show -P LoadState "$unit") || return 1
        if [[ $loaded != not-found ]]; then
            printf 'install.sh: retired unit %s must be uninstalled (LoadState=%s); nothing replaced\n' "$unit" "$loaded" >&2
            return 1
        fi
    done
}

license_paths_safe() {
    local repo=$1 name path
    for path in "$license_dir" "$project_license_dir"; do
        [[ ! -L $path && ( ! -e $path || -d $path ) ]] ||
            fail "unsafe license directory: $path"
    done
    for name in OFL.txt NOTICE.md; do
        [[ -f $repo/crates/hwd/assets/$name && ! -L $repo/crates/hwd/assets/$name ]] ||
            fail "missing bundled font notice: $name"
        [[ ! -L $license_dir/$name && ( ! -e $license_dir/$name || -f $license_dir/$name ) ]] ||
            fail "unsafe existing font notice: $license_dir/$name"
    done
    [[ -f $repo/LICENSE && ! -L $repo/LICENSE ]] || fail 'missing regular project LICENSE'
    [[ ! -L $project_license_dir/LICENSE &&
       ( ! -e $project_license_dir/LICENSE || -f $project_license_dir/LICENSE ) ]] ||
        fail "unsafe existing project license: $project_license_dir/LICENSE"
}

stage=
backup_dir=
replacement_started=0
notice_replaced=0
project_license_replaced=0
completed=0

# The old service binary is required by ExecStopPost until systemd has fully
# stopped it. A failed install rolls back only after verifying it is stopped;
# it never auto-restarts a possibly unsafe fan policy on failure.
cleanup() {
    local status=$? current socket_now
    trap - EXIT INT TERM
    if ((status != 0 && replacement_started && !completed)); then
        current=$(systemctl show -P ActiveState "$service" 2>/dev/null) || current=unknown
        if [[ $current != inactive ]]; then
            systemctl stop "$service" >&2 || :
        fi
        socket_now=$(systemctl show -P ActiveState "$socket" 2>/dev/null) || socket_now=unknown
        if [[ $socket_now != inactive ]]; then
            systemctl stop "$socket" >&2 || :
        fi
        current=$(systemctl show -P ActiveState "$service" 2>/dev/null) || current=unknown
        socket_now=$(systemctl show -P ActiveState "$socket" 2>/dev/null) || socket_now=unknown
        if [[ $current == inactive && $socket_now == inactive && -n $stage && -n $backup_dir ]]; then
            if install -m755 "$backup_dir/nzxt-cam-hwd" "$stage/rollback-hwd" &&
               install -m755 "$backup_dir/nzxt-cam-tui" "$stage/rollback-tui" &&
               mv -fT "$stage/rollback-hwd" "$install_dir/nzxt-cam-hwd" &&
               mv -fT "$stage/rollback-tui" "$install_dir/nzxt-cam-tui"; then
                printf 'install.sh: old binaries restored; units left stopped. Inspect recovery and restart manually.\n' >&2
            else
                printf 'install.sh: automatic rollback failed; old binaries are in %s. Keep the service stopped.\n' "$backup_dir" >&2
            fi
            if ((notice_replaced)); then
                for name in OFL.txt NOTICE.md; do
                    if [[ -f $backup_dir/$name ]]; then
                        install -Dm644 "$backup_dir/$name" "$license_dir/$name" ||
                            printf 'install.sh: could not restore %s; backup is at %s\n' "$name" "$backup_dir" >&2
                    else
                        rm -f -- "$license_dir/$name" ||
                            printf 'install.sh: could not remove new %s\n' "$license_dir/$name" >&2
                    fi
                done
            fi
            if ((project_license_replaced)); then
                if [[ -f $backup_dir/LICENSE ]]; then
                    install -Dm644 "$backup_dir/LICENSE" "$project_license_dir/LICENSE" ||
                        printf 'install.sh: could not restore project LICENSE; backup is at %s\n' "$backup_dir" >&2
                else
                    rm -f -- "$project_license_dir/LICENSE" ||
                        printf 'install.sh: could not remove new %s/LICENSE\n' "$project_license_dir" >&2
                fi
            fi
        else
            printf 'install.sh: service/socket not confirmed stopped; do NOT overwrite binaries. Old binaries: %s\n' "$backup_dir" >&2
        fi
    fi
    if ((status != 0 && !replacement_started)) && [[ -n $backup_dir ]]; then
        printf 'install.sh: no binaries replaced; units may be stopped. Inspect their state before restarting. Backup: %s\n' "$backup_dir" >&2
    fi
    if [[ -n $stage ]]; then rm -rf -- "$stage"; fi
    exit "$status"
}

main() {
    local dry_run=0 repo release name service_state socket_state answer
    case $#:$* in
        0:) ;;
        1:--dry-run) dry_run=1 ;;
        *) printf 'Usage: sudo scripts/install.sh [--dry-run]\n' >&2; exit 2 ;;
    esac
    if (( !dry_run && EUID != 0 )); then
        fail 'run with sudo; build as your regular user first'
    fi
    repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
    release=$repo/target/release
    for name in nzxt-cam-hwd nzxt-cam-tui; do
        [[ -f $release/$name && -x $release/$name && ! -L $release/$name ]] ||
            fail "missing prebuilt $release/$name; first run: scripts/build-release.sh"
        [[ -f $install_dir/$name && ! -L $install_dir/$name ]] ||
            fail "expected an existing regular $install_dir/$name; follow README initial setup first"
    done
    license_paths_safe "$repo"
    source "$repo/scripts/parse-unit-post-status.sh"
    retired_units_absent || fail 'retired units must be absent; nothing changed'
    service_state=$(unit_state "$service")
    socket_state=$(unit_state "$socket")
    if ((dry_run)); then
        printf 'Prebuilt binaries: %s/{nzxt-cam-hwd,nzxt-cam-tui}\n' "$release"
        printf 'Current units: %s=%s, %s=%s\n' "$service" "$service_state" "$socket" "$socket_state"
        printf 'Dry run only: would back up installed binaries and licenses, install both binaries, project LICENSE and font notices, and restart only previously active units.\n'
        if needs_confirmation "$service_state" "$socket_state"; then
            printf 'Would require interactive confirmation before stopping active units.\n'
        fi
        return
    fi

    if needs_confirmation "$service_state" "$socket_state"; then
        [[ -r /dev/tty && -w /dev/tty ]] || fail 'active service/socket requires an interactive terminal'
        confirmation_prompt >/dev/tty
        IFS= read -r answer </dev/tty || fail 'confirmation was not received; nothing changed'
        confirmed_choice "$answer" || fail 'cancelled; nothing changed'
    fi

    perform_install "$repo" "$release" "$service_state" "$socket_state"
}

# Separated from privileged preflight so the stop/install/rollback sequence can
# be exercised with fake systemd and a temporary filesystem in tests.
perform_install() {
    local repo=$1 release=$2 service_state=$3 socket_state=$4 name post
    retired_units_absent || fail 'retired units must be absent; nothing changed'
    license_paths_safe "$repo"
    [[ -d $backup_parent && ! -L $backup_parent ]] || mkdir -m700 -p -- "$backup_parent"
    [[ ! -L $backup_parent && $(stat -c %u "$backup_parent") == 0 &&
       $(stat -c %a "$backup_parent") == 700 ]] ||
        fail "backup directory $backup_parent must be root-owned, private (0700), and not a symlink"
    backup_dir=$(mktemp -d "$backup_parent/install-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXXXX")
    stage=$(mktemp -d "$install_dir/.nzxt-cam-install.XXXXXXXX")
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM

    for name in nzxt-cam-hwd nzxt-cam-tui; do
        install -m755 "$install_dir/$name" "$backup_dir/$name"
        install -m755 "$release/$name" "$stage/new-$name"
    done
    for name in OFL.txt NOTICE.md; do
        if [[ -f $license_dir/$name ]]; then
            install -m644 "$license_dir/$name" "$backup_dir/$name"
        fi
        install -m644 "$repo/crates/hwd/assets/$name" "$stage/new-$name"
    done
    if [[ -f $project_license_dir/LICENSE ]]; then
        install -m644 "$project_license_dir/LICENSE" "$backup_dir/LICENSE"
    fi
    install -m644 "$repo/LICENSE" "$stage/new-LICENSE"

    if [[ $service_state == active ]]; then
        systemctl stop "$service" || fail 'service stop failed; installed binaries were NOT replaced'
    fi
    if [[ $socket_state == active ]]; then
        systemctl stop "$socket" || fail 'socket stop failed; installed binaries were NOT replaced'
    fi
    [[ $(unit_state "$service") == inactive && $(unit_state "$socket") == inactive ]] ||
        fail 'units did not stop cleanly; installed binaries were NOT replaced'
    # systemd may clear ExecStopPost's exit fields once the unit is inactive.
    # For a service already stopped on entry, Result=success is only its
    # last-known result, not proof of BIOS fan ownership. Require the exact
    # configured command and clear recovery state in either case.
    post=$(systemctl show -P ExecStopPost "$service") ||
        fail 'cannot inspect ExecStopPost; installed binaries were NOT replaced'
    normal_service_stop_verified "$post" "$(unit_state "$service")" \
        "$(systemctl show -P Result "$service")" ||
        fail 'ExecStopPost was not verified successful; installed binaries were NOT replaced'
    recovery_clear "$runtime" "$policy_dir" ||
        fail 'installed binaries were NOT replaced'
    retired_units_absent || fail 'a retired unit reappeared; installed binaries were NOT replaced'
    [[ $(unit_state "$service") == inactive && $(unit_state "$socket") == inactive ]] ||
        fail 'service/socket reactivated during preflight; installed binaries were NOT replaced'

    license_paths_safe "$repo"
    replacement_started=1
    mv -fT "$stage/new-nzxt-cam-hwd" "$install_dir/nzxt-cam-hwd" ||
        fail 'could not replace service binary'
    mv -fT "$stage/new-nzxt-cam-tui" "$install_dir/nzxt-cam-tui" ||
        fail 'could not replace TUI binary'
    notice_replaced=1
    for name in OFL.txt NOTICE.md; do
        install -Dm644 "$stage/new-$name" "$license_dir/$name" ||
            fail "could not install font notice $name"
        cmp -s "$stage/new-$name" "$license_dir/$name" ||
            fail "font notice readback differs: $name"
    done
    project_license_replaced=1
    install -Dm644 "$stage/new-LICENSE" "$project_license_dir/LICENSE" ||
        fail 'could not install project LICENSE'
    cmp -s "$stage/new-LICENSE" "$project_license_dir/LICENSE" ||
        fail 'project LICENSE readback differs'
    if needs_confirmation "$service_state" "$socket_state"; then
        systemctl start "$socket" || fail 'could not restart service socket'
    fi
    if [[ $service_state == active ]]; then
        systemctl start "$service" || fail 'could not restart hardware service'
    fi
    [[ $service_state != active || $(unit_state "$service") == active ]] ||
        fail 'new service did not start; reverting binaries'
    [[ $socket_state != active || $(unit_state "$socket") == active ]] ||
        fail 'socket did not restart; reverting binaries'
    completed=1
    installation_success_message "$service_state"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    main "$@"
fi
