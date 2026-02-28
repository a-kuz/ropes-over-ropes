#!/bin/bash
set -e

cd "$(dirname "$0")/.."

export PATH="$HOME/.cargo/bin:$HOME/.rustup/toolchains/stable-aarch64-apple-darwin/bin:$PATH"

cargo build --release --target wasm32-unknown-unknown --bin uzls

mkdir -p web
mkdir -p dist

wasm-bindgen \
    target/wasm32-unknown-unknown/release/uzls.wasm \
    --out-dir web \
    --target web \
    --no-typescript

cp index.html web/

cp web/uzls.js dist/
cp web/uzls_bg.wasm dist/
cp index.html dist/
cp dist/vercel.json dist/vercel.json 2>/dev/null || true

echo "Build complete! WASM files are in cross/web/ and cross/dist/"
echo "To run locally: cd web && python3 -m http.server 8004"
echo "Then open http://localhost:8004"
