#!/usr/bin/env bash

override_env_file=/opt/ws/.devcontainer/override/.env

# Values below are defaults. You can override them by creating a file in .devcontainer/override/.env
# File will be sourced here.

if [ -f "$override_env_file" ]; then
    echo "Sourcing override environment file: $override_env_file"
    source "$override_env_file"
else
    echo "No override environment file found at: $override_env_file"
    echo "Using default values."

    source /opt/ws/.devcontainer/default.env
fi

echo "Map file is expected in $OM_MAP_PATH - $OM_DATUM_LAT, $OM_DATUM_LONG"