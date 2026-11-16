#!/usr/bin/env bash
# Builds dora's cxx node API at the pinned tag once, then lk-dora-host against it (CONTRACT §2.2, §6.4).
#
#   nice -n 19 ionice -c3 /home/bam/like_keras/cpp/hosts/build_dora_cxx.sh
#
# lk-dora-host is the only C++ that links dora; component .so files never do. The cxx crate (apis/c++/node at git
# tag v0.5.0) is fetched and compiled into $LK_DORA_CXX_DIR (default ~/.cache/like_keras/dora-cxx-0.5.0, about
# 0.7 GB, shared by every checkout); a second run only relinks the host. The host lands in <repo>/build/hosts/.
#   LK_DORA_CXX_DIR   where the crate's source, headers and static library go
#   CARGO_TARGET_DIR  cargo's build directory (default $LK_DORA_CXX_DIR/target)
#   JOBS              compile jobs (default 2: the rack is powered on this machine)
set -euo pipefail

VERSION=0.5.0
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PREFIX="${LK_DORA_CXX_DIR:-$HOME/.cache/like_keras/dora-cxx-$VERSION}"
JOBS="${JOBS:-2}"
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$PREFIX/target}"
SOURCE="$PREFIX/src"
LIBRARY="$PREFIX/lib/libdora_node_api_cxx.a"

if [[ ! -f "$LIBRARY" || ! -f "$PREFIX/include/dora-node-api.h" ]]; then
  mkdir -p "$PREFIX/lib" "$SOURCE"
  if [[ ! -f "$SOURCE/Cargo.toml" ]]; then
    # WHY a tag tarball, not a clone: the tag's tree is all the build needs (the root disk is 96 % full)
    curl --fail --location --silent --show-error "https://github.com/dora-rs/dora/archive/refs/tags/v$VERSION.tar.gz" \
      | tar -xz -C "$SOURCE" --strip-components=1
  fi
  # WHY --locked: the tag's own Cargo.lock, so every lane links the same dora 0.5.0
  DORA_NODE_API_CXX_INSTALL="$PREFIX/include" cargo build --manifest-path "$SOURCE/Cargo.toml" --release --locked \
    -p dora-node-api-cxx -j "$JOBS"
  cp "$CARGO_TARGET_DIR/release/libdora_node_api_cxx.a" "$LIBRARY"
fi

mkdir -p "$REPO/build/hosts"
g++ -std=c++20 -O2 -Wall -Wextra -pthread -I"$PREFIX/include" -I"$REPO/cpp/include" \
  "$REPO/cpp/hosts/dora_host.cpp" "$PREFIX/include/dora-node-api.cc" \
  "$LIBRARY" -ldl -lm -lrt -o "$REPO/build/hosts/lk-dora-host.new"
mv "$REPO/build/hosts/lk-dora-host.new" "$REPO/build/hosts/lk-dora-host"
echo "built $REPO/build/hosts/lk-dora-host against dora's cxx node API $VERSION ($LIBRARY)"
