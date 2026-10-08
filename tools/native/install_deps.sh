#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEPS="${NUBOTS_NATIVE_DEPS:-$ROOT/build-deps}"
PREFIX="$DEPS/install"
JOBS="${JOBS:-8}"
mkdir -p "$DEPS/src" "$PREFIX"

build() {
    local name=$1 version=$2 url=$3
    shift 3
    local source="$DEPS/src/$name-$version"
    if [ -f "$PREFIX/.stamp-$name-$version" ]; then
        echo "== $name $version (cached)"
        return
    fi
    if [ ! -d "$source" ]; then
        mkdir -p "$source"
        curl -fL "$url" | tar -xz --strip-components=1 -C "$source"
    fi
    if [ "$name" = nuclear ]; then
        # Strict Clang requires a template argument list at these dependent calls.
        python3 - "$source/src/dsl/fusion/BindFusion.hpp" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
path.write_text(path.read_text().replace(">::template call(", ">::template call<>("))
PY
    fi
    cmake -S "$source" -B "$source/build" -GNinja \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX;/opt/homebrew;/usr/local" \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$@"
    cmake --build "$source/build" --parallel "$JOBS"
    cmake --install "$source/build"
    touch "$PREFIX/.stamp-$name-$version"
}

# Match the deployed repository's pins rather than Homebrew's newer ABIs.
build protobuf 3.21.4 \
    https://github.com/protocolbuffers/protobuf/releases/download/v21.4/protobuf-cpp-3.21.4.tar.gz \
    -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_BUILD_LIBPROTOC=ON -DBUILD_SHARED_LIBS=OFF
build fmt 9.0.0 https://github.com/fmtlib/fmt/archive/refs/tags/9.0.0.tar.gz \
    -DFMT_TEST=OFF -DFMT_DOC=OFF
build eigen 3.4.0 https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    -DBUILD_TESTING=OFF
build nuclear 925dca0f31484a7df64fd335de5a6c9335483c7f \
    https://github.com/Fastcode/NUClear/archive/925dca0f31484a7df64fd335de5a6c9335483c7f.tar.gz \
    -DBUILD_TESTS=OFF
build uv 1.44.2 https://github.com/libuv/libuv/archive/refs/tags/v1.44.2.tar.gz \
    -Dlibuv_buildtests=OFF -DBUILD_TESTING=OFF

if [ ! -f "$PREFIX/.stamp-onnxruntime-1.24.1" ]; then
    case "$(uname -s)-$(uname -m)" in
        Darwin-arm64) ORT_ARCHIVE=onnxruntime-osx-arm64-1.24.1.tgz ;;
        *) echo "This installer currently targets Apple Silicon macOS." >&2; exit 1 ;;
    esac
    curl -fL "https://github.com/microsoft/onnxruntime/releases/download/v1.24.1/$ORT_ARCHIVE" \
        -o "$DEPS/onnxruntime.tgz"
    mkdir -p "$DEPS/src/onnxruntime"
    tar -xzf "$DEPS/onnxruntime.tgz" --strip-components=1 -C "$DEPS/src/onnxruntime"
    cp -R "$DEPS/src/onnxruntime/include/." "$PREFIX/include/"
    cp -R "$DEPS/src/onnxruntime/lib/." "$PREFIX/lib/"
    touch "$PREFIX/.stamp-onnxruntime-1.24.1"
fi

if [ ! -x "$DEPS/venv/bin/python" ]; then
    uv venv --python 3.11 "$DEPS/venv"
fi
uv pip install --python "$DEPS/venv/bin/python" \
    protobuf==5.27.0 casefy==1.1.0 pyyaml==6.0.2 pillow==11.1.0 xxhash==3.5.0
echo "Native player dependencies installed in $PREFIX"
