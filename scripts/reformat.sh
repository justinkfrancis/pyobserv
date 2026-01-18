#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
TARGET_DIR="$SCRIPT_DIR/../pyobserv"
EXAMPLE_DIR="$SCRIPT_DIR/../examples"
TEST_DIR="$SCRIPT_DIR/../tests"

black "$TARGET_DIR" "$TEST_DIR" "$EXAMPLE_DIR" --safe --line-length=78 --target-version=py312
