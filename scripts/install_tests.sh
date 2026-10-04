#!/usr/bin/env bash
# Pure installer guard tests: no systemd calls, root access or hardware writes.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/install.sh

for state in active inactive; do
    acceptable_unit_state "$state"
done
for state in activating deactivating failed unknown; do
    if acceptable_unit_state "$state"; then
        echo "installer accepted transitional/failed unit state: $state" >&2; exit 1
    fi
done
for states in 'active inactive' 'inactive active' 'active active'; do
    read -r first second <<<"$states"
    needs_confirmation "$first" "$second"
done
if needs_confirmation inactive inactive; then
    echo 'installer would prompt despite no active units' >&2; exit 1
fi
expected_prompt=$(printf '%s\n' \
    'The NZXT hardware service/socket will STOP for installation.' \
    'Stopping can interrupt active fan control and return fans to BIOS control.' \
    'Proceed?(Y/n)')
[[ $(confirmation_prompt) == "$expected_prompt" ]] || {
    echo 'installer confirmation prompt changed unexpectedly' >&2; exit 1;
}
for answer in '' y Y yes YES; do
    confirmed_choice "$answer" || {
        echo "installer rejected affirmative/default confirmation: $answer" >&2; exit 1;
    }
done
for answer in n N no NO maybe ' '; do
    if confirmed_choice "$answer"; then
        echo "installer accepted negative/ambiguous confirmation: $answer" >&2; exit 1
    fi
done

source scripts/parse-unit-post-status.sh
cleared_post='{ path=/usr/bin/nzxt-cam-hwd ; argv[]=/usr/bin/nzxt-cam-hwd recover-service ; ignore_errors=no ; start_time=[n/a] ; stop_time=[n/a] ; pid=0 ; code=(null) ; status=0/0 }'
succeeded_post='{ path=/usr/bin/nzxt-cam-hwd ; argv[]=/usr/bin/nzxt-cam-hwd recover-service ; ignore_errors=no ; start_time=now ; stop_time=now ; pid=123 ; code=exited ; status=0/SUCCESS }'
normal_service_stop_verified "$cleared_post" inactive success
normal_service_stop_verified "$succeeded_post" inactive success
for bad_post in \
    '' \
    '{ code=exited ; status=0/SUCCESS }' \
    "${cleared_post/recover-service/restore}" \
    "${cleared_post/ignore_errors=no/ignore_errors=yes}" \
    "${cleared_post/status=0\/0/status=1\/FAILURE}" \
    "${succeeded_post/status=0\/SUCCESS/status=1\/FAILURE}" \
    "$succeeded_post { code=exited ; status=1/FAILURE }"; do
    if normal_service_stop_verified "$bad_post" inactive success; then
        echo "installer accepted unverified normal post-stop status: $bad_post" >&2; exit 1
    fi
done
for bad_state in active failed; do
    if normal_service_stop_verified "$cleared_post" "$bad_state" success; then
        echo "installer accepted post-stop with $bad_state service" >&2; exit 1
    fi
done
if normal_service_stop_verified "$cleared_post" inactive exit-code; then
    echo 'installer accepted failed service result' >&2; exit 1
fi
mock_unit= mock_load=not-found
systemctl() {
    [[ $# == 4 && $1 == show && $2 == -P && $3 == LoadState ]] || {
        echo 'retired-unit guard must only inspect LoadState' >&2; return 1;
    }
    if [[ $4 == "$mock_unit" ]]; then
        [[ $mock_load != inspection-error ]] || return 1
        printf '%s\n' "$mock_load"
    else
        printf 'not-found\n'
    fi
}
retired_units_absent
for mock_unit in \
    nzxt-cam-it8689-fan3-handoff.service nzxt-cam-it8689-fan4-handoff.service \
    nzxt-cam-it8689-dual-trial.service nzxt-cam-it8689-fan3-probe.service \
    nzxt-cam-it8689-fan4-probe.service; do
    for mock_load in loaded masked error '' inspection-error; do
        if retired_units_absent 2>/dev/null; then
            echo "installer accepted retired unit $mock_unit with LoadState=$mock_load" >&2; exit 1
        fi
    done
done
mock_load=not-found
retired_units_absent
unset -f systemctl

root=$(mktemp -d)
trap 'rm -rf -- "$root"' EXIT
mkdir -p "$root"/{normal,policy}
recovery_clear "$root/normal" "$root/policy"
printf '{"state":"enabled"}\n' > "$root/policy/active-policy.json"
printf '{"state":"ready"}\n' > "$root/policy/monitor-intent.json"
recovery_clear "$root/normal" "$root/policy"
for path in \
    normal/host-control.json normal/host-control.json.tmp normal/host-control.restored \
    policy/active-policy.json.tmp policy/monitor-intent.json.tmp; do
    touch "$root/$path"
    if recovery_clear "$root/normal" "$root/policy" 2>/dev/null; then
        echo "installer ignored recovery state: $path" >&2; exit 1
    fi
    rm -- "$root/$path"
done
ln -s /nonexistent "$root/normal/host-control.json"
if recovery_clear "$root/normal" "$root/policy" 2>/dev/null; then
    echo 'installer ignored symlinked recovery state' >&2; exit 1
fi
# Exercise the privileged sequence against a temporary filesystem and fake
# systemd. In particular, a failed service restart must close the socket and
# restore both binaries, font notices and project license without touching the host.
prepare_fixture() {
    local base=$1 with_project_license=${2:-yes} name
    mkdir -p "$base/usr/bin" "$base/usr/share/licenses/nzxt-cam-hwd" "$base/target/release"
    for name in nzxt-cam-hwd nzxt-cam-tui; do
        printf 'old-%s\n' "$name" > "$base/usr/bin/$name"
        printf 'new-%s\n' "$name" > "$base/target/release/$name"
        chmod 755 "$base/usr/bin/$name" "$base/target/release/$name"
    done
    printf 'old-license\n' > "$base/usr/share/licenses/nzxt-cam-hwd/OFL.txt"
    printf 'old-notice\n' > "$base/usr/share/licenses/nzxt-cam-hwd/NOTICE.md"
    if [[ $with_project_license == yes ]]; then
        mkdir -p "$base/usr/share/licenses/nzxt-cam-tui"
        printf 'old-project-license\n' > "$base/usr/share/licenses/nzxt-cam-tui/LICENSE"
    fi
    printf 'active\n' > "$base/service-state"
    printf 'active\n' > "$base/socket-state"
}

fake_upgrade() (
    local base=$1 fail_start=$2 post_mode=${3:-cleared} starting_service=${4:-active} starting_socket=${5:-active} retired_load=${6:-not-found} property unit state_file
    source scripts/install.sh
    source scripts/parse-unit-post-status.sh
    install_dir=$base/usr/bin
    license_dir=$base/usr/share/licenses/nzxt-cam-hwd
    project_license_dir=$base/usr/share/licenses/nzxt-cam-tui
    backup_parent=$base/var/backups/nzxt-cam
    runtime=$base/run/normal
    policy_dir=$base/var/lib/nzxt-cam
    stat() {
        if [[ $1 == -c && $2 == %u && $3 == "$backup_parent" ]]; then
            printf '0\n' # simulate a root-owned private backup directory
        else
            command stat "$@"
        fi
    }
    install() {
        if [[ $fail_start == notice && ${*: -1} == "$license_dir/NOTICE.md" &&
              ! -e $base/notice-failed ]]; then
            touch "$base/notice-failed"
            return 1
        fi
        if [[ ( $fail_start == project-license || $fail_start == license-readback ) &&
              ${*: -1} == "$project_license_dir/LICENSE" && ! -e $base/license-failed ]]; then
            touch "$base/license-failed"
            command install "$@" || return 1
            # Simulate a copy with side effects before failure or bad readback.
            [[ $fail_start != project-license ]] || return 1
            printf 'corrupted-license\n' > "$project_license_dir/LICENSE"
            return 0
        fi
        command install "$@"
    }
    systemctl() {
        case $1 in
            show)
                property=$3 unit=$4
                case $property in
                    LoadState)
                        if [[ $unit == "$service" || $unit == "$socket" ]]; then
                            echo loaded
                        else
                            echo "$retired_load"
                        fi ;;
                    ActiveState)
                        if [[ $unit == "$service" ]]; then cat "$base/service-state"
                        elif [[ $unit == "$socket" ]]; then cat "$base/socket-state"
                        else echo inactive; fi ;;
                    Result)
                        if [[ $unit == "$service" && $post_mode == failed-result ]]; then
                            echo exit-code
                        else
                            echo success
                        fi ;;
                    ExecStopPost)
                        if [[ $unit == "$service" ]]; then
                            if [[ $post_mode == wrong-command ]]; then
                                echo "${cleared_post/recover-service/restore}"
                            else
                                echo "$cleared_post"
                            fi
                        else
                            echo '{ code=exited ; status=0/SUCCESS }'
                        fi ;;
                    *) return 1 ;;
                esac ;;
            stop)
                [[ $# == 2 ]] || return 1
                if [[ $2 == "$service" ]]; then state_file=$base/service-state
                elif [[ $2 == "$socket" ]]; then state_file=$base/socket-state
                else return 1; fi
                printf 'inactive\n' > "$state_file"
                [[ $post_mode != stop-failed || $2 != "$service" ]] ;;
            start)
                if [[ $2 == "$service" ]]; then
                    if [[ $fail_start == yes ]]; then
                        printf 'failed\n' > "$base/service-state"
                        return 1
                    fi
                    state_file=$base/service-state
                elif [[ $2 == "$socket" ]]; then state_file=$base/socket-state
                else return 1; fi
                printf 'active\n' > "$state_file" ;;
            *) return 1 ;;
        esac
    }
    perform_install "$PWD" "$base/target/release" "$starting_service" "$starting_socket"
)

stale=$root/stale-retired-unit
prepare_fixture "$stale"
if fake_upgrade "$stale" no cleared active active loaded >"$root/stale.log" 2>&1; then
    echo 'installer accepted an installed retired unit' >&2; exit 1
fi
for name in nzxt-cam-hwd nzxt-cam-tui; do
    [[ $(cat "$stale/usr/bin/$name") == "old-$name" ]] || {
        echo "installer replaced $name despite an installed retired unit" >&2; exit 1;
    }
done
[[ ! -e $stale/var/backups/nzxt-cam &&
   $(cat "$stale/service-state") == active && $(cat "$stale/socket-state") == active ]] || {
    echo 'retired-unit rejection must precede backup and service/socket stops' >&2; exit 1;
}

success=$root/success
prepare_fixture "$success"
fake_upgrade "$success" no >"$root/success.log"
[[ $(cat "$root/success.log") == $'NZXT CAM installed successfully.\nHardware service is running.' ]] || {
    echo 'installer did not print the concise running-service success message' >&2; exit 1;
}
for name in nzxt-cam-hwd nzxt-cam-tui; do
    cmp -s "$success/usr/bin/$name" "$success/target/release/$name"
done
cmp -s "$success/usr/share/licenses/nzxt-cam-hwd/OFL.txt" crates/hwd/assets/OFL.txt
cmp -s "$success/usr/share/licenses/nzxt-cam-hwd/NOTICE.md" crates/hwd/assets/NOTICE.md
cmp -s "$success/usr/share/licenses/nzxt-cam-tui/LICENSE" LICENSE
for path in nzxt-cam-hwd/OFL.txt nzxt-cam-hwd/NOTICE.md nzxt-cam-tui/LICENSE; do
    [[ $(stat -c %a "$success/usr/share/licenses/$path") == 644 ]]
done
[[ $(cat "$success/service-state") == active && $(cat "$success/socket-state") == active ]]

already_stopped=$root/already-stopped
prepare_fixture "$already_stopped" no
printf 'inactive\n' >"$already_stopped/service-state"
printf 'inactive\n' >"$already_stopped/socket-state"
fake_upgrade "$already_stopped" no cleared inactive inactive >"$root/stopped-success.log"
[[ $(cat "$root/stopped-success.log") == $'NZXT CAM installed successfully.\nTo start the hardware service when ready:\n  sudo systemctl start nzxt-cam-hwd.service' ]] || {
    echo 'installer did not print the service start command after a stopped install' >&2; exit 1;
}
for name in nzxt-cam-hwd nzxt-cam-tui; do
    cmp -s "$already_stopped/usr/bin/$name" "$already_stopped/target/release/$name"
done
cmp -s "$already_stopped/usr/share/licenses/nzxt-cam-tui/LICENSE" LICENSE
[[ $(stat -c %a "$already_stopped/usr/share/licenses/nzxt-cam-tui/LICENSE") == 644 ]]
[[ $(cat "$already_stopped/service-state") == inactive &&
   $(cat "$already_stopped/socket-state") == inactive ]] || {
    echo 'installer restarted already-stopped units' >&2; exit 1;
}

failure=$root/failure
prepare_fixture "$failure"
if fake_upgrade "$failure" yes >"$root/failure.log" 2>&1; then
    echo 'installer accepted a failed service restart' >&2; exit 1
fi
for name in nzxt-cam-hwd nzxt-cam-tui; do
    [[ $(cat "$failure/usr/bin/$name") == "old-$name" ]] || {
        echo "installer did not roll back $name" >&2; exit 1;
    }
done
[[ $(cat "$failure/usr/share/licenses/nzxt-cam-hwd/OFL.txt") == old-license &&
   $(cat "$failure/usr/share/licenses/nzxt-cam-hwd/NOTICE.md") == old-notice &&
   $(cat "$failure/usr/share/licenses/nzxt-cam-tui/LICENSE") == old-project-license &&
   $(cat "$failure/service-state") == inactive && $(cat "$failure/socket-state") == inactive ]] || {
    echo 'installer did not restore notices or stop units after failure' >&2; exit 1;
}

for post_mode in wrong-command failed-result; do
    blocked=$root/$post_mode
    prepare_fixture "$blocked"
    if fake_upgrade "$blocked" no "$post_mode" >"$root/$post_mode.log" 2>&1; then
        echo "installer ignored unsafe post-stop state: $post_mode" >&2; exit 1
    fi
    for name in nzxt-cam-hwd nzxt-cam-tui; do
        [[ $(cat "$blocked/usr/bin/$name") == "old-$name" ]] || {
            echo "installer replaced $name despite $post_mode" >&2; exit 1;
        }
    done
    [[ $(cat "$blocked/service-state") == inactive &&
       $(cat "$blocked/socket-state") == inactive ]] || {
        echo "installer restarted units despite $post_mode" >&2; exit 1;
    }
done

blocked_inactive=$root/blocked-inactive
prepare_fixture "$blocked_inactive"
printf 'inactive\n' >"$blocked_inactive/service-state"
printf 'inactive\n' >"$blocked_inactive/socket-state"
if fake_upgrade "$blocked_inactive" no failed-result inactive inactive >"$root/blocked-inactive.log" 2>&1; then
    echo 'installer accepted failed result for already-stopped service' >&2; exit 1
fi
for name in nzxt-cam-hwd nzxt-cam-tui; do
    [[ $(cat "$blocked_inactive/usr/bin/$name") == "old-$name" ]] || {
        echo "installer replaced $name despite failed inactive result" >&2; exit 1;
    }
done

stopping_failure=$root/stop-failed
prepare_fixture "$stopping_failure"
if fake_upgrade "$stopping_failure" no stop-failed >"$root/stop-failed.log" 2>&1; then
    echo 'installer ignored a failed systemctl stop' >&2; exit 1
fi
for name in nzxt-cam-hwd nzxt-cam-tui; do
    [[ $(cat "$stopping_failure/usr/bin/$name") == "old-$name" ]] || {
        echo "installer replaced $name despite failed stop" >&2; exit 1;
    }
done

notice_failure=$root/notice-failure
prepare_fixture "$notice_failure"
if fake_upgrade "$notice_failure" notice >"$root/notice-failure.log" 2>&1; then
    echo 'installer accepted a failed font notice copy' >&2; exit 1
fi
for name in nzxt-cam-hwd nzxt-cam-tui; do
    [[ $(cat "$notice_failure/usr/bin/$name") == "old-$name" ]] || {
        echo "installer did not roll back $name after notice failure" >&2; exit 1;
    }
done
[[ $(cat "$notice_failure/usr/share/licenses/nzxt-cam-hwd/OFL.txt") == old-license &&
   $(cat "$notice_failure/usr/share/licenses/nzxt-cam-hwd/NOTICE.md") == old-notice &&
   $(cat "$notice_failure/usr/share/licenses/nzxt-cam-tui/LICENSE") == old-project-license &&
   $(cat "$notice_failure/service-state") == inactive && $(cat "$notice_failure/socket-state") == inactive ]] || {
    echo 'installer did not roll back notices or units after notice failure' >&2; exit 1;
}
# Rollback must restore a prior project license, or remove only the new file
# when no project license existed, including an ambiguous copy/readback failure.
for prior in yes no; do
    for failure_mode in yes project-license license-readback; do
        base=$root/project-license-$prior-$failure_mode
        prepare_fixture "$base" "$prior"
        if fake_upgrade "$base" "$failure_mode" >"$base/log" 2>&1; then
            echo "installer ignored $failure_mode with prior project license=$prior" >&2; exit 1
        fi
        for name in nzxt-cam-hwd nzxt-cam-tui; do
            [[ $(cat "$base/usr/bin/$name") == "old-$name" ]]
        done
        if [[ $prior == yes ]]; then
            [[ $(cat "$base/usr/share/licenses/nzxt-cam-tui/LICENSE") == old-project-license &&
               $(stat -c %a "$base/usr/share/licenses/nzxt-cam-tui/LICENSE") == 644 ]]
        else
            [[ ! -e $base/usr/share/licenses/nzxt-cam-tui/LICENSE &&
               ! -L $base/usr/share/licenses/nzxt-cam-tui/LICENSE ]]
        fi
        [[ $(cat "$base/usr/share/licenses/nzxt-cam-hwd/OFL.txt") == old-license &&
           $(cat "$base/usr/share/licenses/nzxt-cam-hwd/NOTICE.md") == old-notice &&
           $(cat "$base/service-state") == inactive && $(cat "$base/socket-state") == inactive ]]
    done
done

guard_root=$root/license-guards
mkdir -p "$guard_root/font" "$guard_root/project"
check_license_paths() (
    license_dir=$guard_root/font
    project_license_dir=$guard_root/project
    license_paths_safe "$PWD"
)
check_license_paths
ln -s /nonexistent "$guard_root/project/LICENSE"
if check_license_paths 2>/dev/null; then
    echo 'installer accepted a symlinked project license' >&2; exit 1
fi
[[ -L $guard_root/project/LICENSE ]]
rm -- "$guard_root/project/LICENSE"
mkdir "$guard_root/project/LICENSE"
if check_license_paths 2>/dev/null; then
    echo 'installer accepted a non-regular project license' >&2; exit 1
fi
rmdir "$guard_root/project/LICENSE" "$guard_root/project"
ln -s "$guard_root/font" "$guard_root/project"
if check_license_paths 2>/dev/null; then
    echo 'installer accepted a symlinked project license directory' >&2; exit 1
fi
[[ -L $guard_root/project && -d $guard_root/font ]]
printf 'install.sh guards and isolated upgrade/rollback: ok\n'
