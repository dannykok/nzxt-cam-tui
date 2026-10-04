#!/usr/bin/env bash
# Build the matching service/TUI pair without embedding private build paths.
set -euo pipefail
(( EUID != 0 )) || { echo 'Build as your regular user, not root.' >&2; exit 1; }
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cd -- "$repo"
[[ ${HOME:-} == /* && $HOME != / ]] || {
    echo 'An absolute, non-root HOME is required for source-path remapping.' >&2; exit 1;
}
home=${HOME%/}
cargo_home=${CARGO_HOME:-"$home/.cargo"}
[[ $cargo_home == /* ]] || cargo_home=$repo/$cargo_home
cargo_home=${cargo_home%/}

separator=$'\x1f'
if [[ ${CARGO_ENCODED_RUSTFLAGS+x} ]]; then
    flags=$CARGO_ENCODED_RUSTFLAGS
else
    flags=
    # Match Cargo's whitespace-separated RUSTFLAGS convention. Encoded flags
    # remain available for arguments containing spaces.
    if [[ -n ${RUSTFLAGS:-} ]]; then
        read -r -a existing <<< "$RUSTFLAGS"
        for flag in "${existing[@]}"; do
            flags+=${flags:+$separator}$flag
        done
    fi
fi
# Last matching prefix wins: retain useful neutral crate/workspace locations.
for mapping in "$home=/build/user" "$cargo_home=/build/cargo" "$repo=/build/workspace"; do
    flags+=${flags:+$separator}"--remap-path-prefix=$mapping"
done
export CARGO_ENCODED_RUSTFLAGS=$flags
exec cargo build --release --locked -p nzxt-cam-hwd -p nzxt-cam-tui --bins "$@"
