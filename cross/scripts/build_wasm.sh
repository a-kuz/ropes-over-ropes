#!/bin/bash
set -e

cd "$(dirname "$0")/.."

export PATH="$HOME/.cargo/bin:$HOME/.rustup/toolchains/stable-aarch64-apple-darwin/bin:$PATH"

cargo build --release --target wasm32-unknown-unknown --bin uzls

mkdir -p web

wasm-bindgen \
    target/wasm32-unknown-unknown/release/uzls.wasm \
    --out-dir web \
    --target web \
    --no-typescript

cp index.html web/

echo "Build complete! WASM files are in cross/web/"
echo "To run: cd web && python3 -m http.server 8004"
echo "Then open http://localhost:8004"
