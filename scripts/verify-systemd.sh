#!/usr/bin/env bash
# Fake and staged packaging checks only: never contact live systemd or sysfs.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/parse-unit-post-status.sh

for status in 0 0/0 0/SUCCESS; do
    post="{ path=/usr/bin/nzxt-cam-hwd ; argv[]=/usr/bin/nzxt-cam-hwd recover-service ; ignore_errors=no ; code=exited ; status=$status }"
    unit_post_succeeded "$post"
    normal_service_stop_verified "$post" inactive success
done
for post in '' '{ code=(null) ; status=0/0 }' '{ code=killed ; status=0 }' \
    '{ code=exited ; status=01 }' '{ code=exited ; status=1/FAILURE }'; do
    if unit_post_succeeded "$post"; then
        echo "Accepted failed post status: $post" >&2; exit 1
    fi
done

for script in scripts/install.sh scripts/install_tests.sh scripts/parse-unit-post-status.sh \
    scripts/build-release.sh scripts/build-release_tests.sh; do
    bash -n "$script"
done
# systemctl --wait is only supported for start/restart, not stop.
if grep -Eq 'systemctl[[:space:]]+--wait[[:space:]]+stop' scripts/install.sh; then
    echo 'Unsupported systemctl --wait stop found in installer' >&2; exit 1
fi
bash scripts/build-release_tests.sh
bash scripts/install_tests.sh

# Stage only production files under an isolated root. No daemon is started,
# and systemd verification below is restricted to this temporary filesystem.
stage=$(mktemp -d)
trap 'rm -rf -- "$stage"' EXIT
cargo build --locked -p nzxt-cam-hwd -p nzxt-cam-tui --bins
for name in nzxt-cam-hwd nzxt-cam-tui; do
    install -Dm755 "target/debug/$name" "$stage/usr/bin/$name"
done
for name in OFL.txt NOTICE.md; do
    install -Dm644 "crates/hwd/assets/$name" "$stage/usr/share/licenses/nzxt-cam-hwd/$name"
done
install -Dm644 LICENSE "$stage/usr/share/licenses/nzxt-cam-tui/LICENSE"
expected_units=$(printf '%s\n' nzxt-cam-hwd.service nzxt-cam-hwd.socket)
packaged_units=$(find packaging/systemd -maxdepth 1 -type f \( -name '*.service' -o -name '*.socket' \) -printf '%f\n' | sort)
[[ $packaged_units == "$expected_units" ]] || {
    echo 'Unexpected production unit inventory' >&2; exit 1;
}
for unit in nzxt-cam-hwd.service nzxt-cam-hwd.socket; do
    install -Dm644 "packaging/systemd/$unit" "$stage/usr/lib/systemd/system/$unit"
done
install -Dm644 packaging/sysusers.d/nzxt-cam.conf "$stage/usr/lib/sysusers.d/nzxt-cam.conf"
install -Dm644 packaging/nzxt-cam-hardware.toml.example \
    "$stage/usr/share/doc/nzxt-cam/hardware.toml.example"
test ! -e "$stage/etc/nzxt-cam/hardware.toml"
if grep -Eq '^[[:space:]]*\[\[?host_control([.]|\])' \
    "$stage/usr/share/doc/nzxt-cam/hardware.toml.example"; then
    echo 'Packaged motherboard-control example must remain commented out' >&2; exit 1
fi
expected_inventory=$(printf '%s\n' \
    usr/bin/nzxt-cam-hwd usr/bin/nzxt-cam-tui \
    usr/lib/systemd/system/nzxt-cam-hwd.service usr/lib/systemd/system/nzxt-cam-hwd.socket \
    usr/lib/sysusers.d/nzxt-cam.conf usr/share/doc/nzxt-cam/hardware.toml.example \
    usr/share/licenses/nzxt-cam-hwd/OFL.txt usr/share/licenses/nzxt-cam-hwd/NOTICE.md \
    usr/share/licenses/nzxt-cam-tui/LICENSE | sort)
inventory=$(find "$stage" -type f -printf '%P\n' | sort)
[[ $inventory == "$expected_inventory" ]] || {
    echo 'Unexpected staged production inventory' >&2; exit 1;
}
cmp -s LICENSE "$stage/usr/share/licenses/nzxt-cam-tui/LICENSE"
for path in nzxt-cam-hwd/OFL.txt nzxt-cam-hwd/NOTICE.md nzxt-cam-tui/LICENSE; do
    test "$(stat -c %a "$stage/usr/share/licenses/$path")" = 644
done
service_unit="$stage/usr/lib/systemd/system/nzxt-cam-hwd.service"
for line in 'ExecStart=/usr/bin/nzxt-cam-hwd' \
    'ExecStopPost=/usr/bin/nzxt-cam-hwd recover-service' 'User=root' 'Group=root' \
    'Requires=nzxt-cam-hwd.socket' 'After=nzxt-cam-hwd.socket' \
    'Sockets=nzxt-cam-hwd.socket' 'StateDirectory=nzxt-cam' \
    'StateDirectoryMode=0700' 'WantedBy=multi-user.target'; do
    test "$(grep -Fxc "$line" "$service_unit")" -eq 1
done
socket_unit="$stage/usr/lib/systemd/system/nzxt-cam-hwd.socket"
for line in 'ListenStream=/run/nzxt-cam/hardware.sock' 'Accept=no' \
    'SocketUser=root' 'SocketGroup=nzxt-cam-control' 'SocketMode=0660' \
    'DirectoryMode=0755' 'RemoveOnStop=true' 'WantedBy=sockets.target'; do
    test "$(grep -Fxc "$line" "$socket_unit")" -eq 1
done
for target in sysinit.target sockets.target basic.target multi-user.target; do
    printf '[Unit]\nDescription=Verification target stub\n' >"$stage/usr/lib/systemd/system/$target"
done
systemd-sysusers --root="$stage"
systemd-analyze --root="$stage" verify nzxt-cam-hwd.socket nzxt-cam-hwd.service
printf 'Production staged inventory and systemd verification: ok\n'
