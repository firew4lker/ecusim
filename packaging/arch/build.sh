#!/usr/bin/env bash
# Build the Arch Linux package from this source tree.
#   ./build.sh          build ecusim-<ver>-<rel>-any.pkg.tar.zst
#   ./build.sh -si      also install missing dependencies and the package (asks for sudo)
# Any arguments are passed on to makepkg (e.g. --nocheck to skip the test suite).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
cd "$here"

pkgver=$(sed -n 's/^pkgver=//p' PKGBUILD)
srcver=$(sed -n 's/^version = "\(.*\)"/\1/p' "$root/pyproject.toml")
if [ "$pkgver" != "$srcver" ]; then
    echo "PKGBUILD pkgver ($pkgver) differs from pyproject.toml version ($srcver)" >&2
    exit 1
fi

# Source tarball from the working tree (virtual environments and build output left out)
tar -czf "ecusim-$pkgver.tar.gz" -C "$root" \
    --exclude='./.venv*' --exclude='./build' --exclude='./dist' --exclude='./out' \
    --exclude='./packaging/arch/src' --exclude='./packaging/arch/pkg' \
    --exclude='./packaging/arch/*.tar.gz' --exclude='./packaging/arch/*.pkg.tar.*' \
    --exclude='__pycache__' --exclude='.pytest_cache' --exclude='*.egg-info' \
    --transform "s,^\.,ecusim-$pkgver," .

# Record the tarball's checksum in the PKGBUILD (pacman-contrib); makepkg verifies it
if command -v updpkgsums >/dev/null; then
    updpkgsums
fi

makepkg -f --cleanbuild "$@"
