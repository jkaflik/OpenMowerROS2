#!/usr/bin/env bash

set -e

SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd "$SCRIPT_PATH/.."

# Install custom dependencies using vcstool

mkdir -p src/lib
vcs import src/lib --force --shallow < custom_deps.yaml
