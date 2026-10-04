#!/usr/bin/env bash
# Exercise release flags with a fake Cargo executable; no compilation or hardware.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
repo=$PWD
root=$(mktemp -d)
trap 'rm -rf -- "$root"' EXIT
mkdir -p "$root/bin" "$root/private home" "$root/private cargo"
cat > "$root/bin/cargo" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\0' "$@" > "$TEST_OUTPUT/args"
printf '%s' "$CARGO_ENCODED_RUSTFLAGS" > "$TEST_OUTPUT/flags"
printf '%s' "$PWD" > "$TEST_OUTPUT/cwd"
exit "${TEST_CARGO_STATUS:-0}"
SH
chmod 755 "$root/bin/cargo"

check_case() {
    local name=$1
    shift
    mkdir -p "$root/$name"
    env -u RUSTFLAGS -u CARGO_ENCODED_RUSTFLAGS \
        HOME="$root/private home" CARGO_HOME="$root/private cargo" \
        PATH="$root/bin:/usr/bin:/bin" TEST_OUTPUT="$root/$name" \
        "$@" bash scripts/build-release.sh --offline
}
check_case plain
check_case ordinary RUSTFLAGS='-C debuginfo=1'
check_case encoded CARGO_ENCODED_RUSTFLAGS=$'--cfg\x1frelease_test="with spaces"' RUSTFLAGS='ignored'
check_case empty CARGO_ENCODED_RUSTFLAGS='' RUSTFLAGS='ignored'
check_case default_cargo env -u CARGO_HOME
check_case relative_cargo CARGO_HOME='relative cargo'
python3 - "$root" "$repo" <<'PY'
import pathlib, sys
root, repo = pathlib.Path(sys.argv[1]), sys.argv[2]
expected_args = ['build', '--release', '--locked', '-p', 'nzxt-cam-hwd', '-p', 'nzxt-cam-tui', '--bins', '--offline']
mappings = [
    f'--remap-path-prefix={root}/private home=/build/user',
    f'--remap-path-prefix={root}/private cargo=/build/cargo',
    f'--remap-path-prefix={repo}=/build/workspace',
]
for name, prefix in [
    ('plain', []), ('ordinary', ['-C', 'debuginfo=1']),
    ('encoded', ['--cfg', 'release_test="with spaces"']), ('empty', []),
    ('default_cargo', []), ('relative_cargo', []),
]:
    expected_mappings = mappings.copy()
    if name == 'default_cargo':
        expected_mappings[1] = f'--remap-path-prefix={root}/private home/.cargo=/build/cargo'
    elif name == 'relative_cargo':
        expected_mappings[1] = f'--remap-path-prefix={repo}/relative cargo=/build/cargo'
    case = root / name
    args = case.joinpath('args').read_bytes().decode().split('\0')[:-1]
    assert args == expected_args, (name, args)
    assert case.joinpath('flags').read_text().split('\x1f') == prefix + expected_mappings, name
    assert case.joinpath('cwd').read_text() == repo, name
PY
mkdir -p "$root/failure"
status=0
env -u RUSTFLAGS -u CARGO_ENCODED_RUSTFLAGS \
    HOME="$root/private home" CARGO_HOME="$root/private cargo" \
    PATH="$root/bin:/usr/bin:/bin" TEST_OUTPUT="$root/failure" TEST_CARGO_STATUS=42 \
    bash scripts/build-release.sh || status=$?
[[ $status == 42 ]] || { echo 'release helper hid the Cargo failure' >&2; exit 1; }
for invalid_home in relative / ''; do
    if env HOME="$invalid_home" PATH="$root/bin:/usr/bin:/bin" TEST_OUTPUT="$root/failure" \
        bash scripts/build-release.sh >/dev/null 2>&1; then
        echo 'release helper accepted an unsafe HOME' >&2; exit 1
    fi
done
printf 'build-release.sh path remapping and argument forwarding: ok\n'
