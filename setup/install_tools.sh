#!/usr/bin/env bash
# Install the build/runtime packages required by a selected environment.
set -euo pipefail

profile="${1:-}"
if (( $# != 1 )); then
    echo "Usage: ./setup/install_tools.sh sitl|hardware" >&2
    exit 2
fi

case "${profile}" in
    sitl|hardware) ;;
    *)
        echo "Usage: ./setup/install_tools.sh sitl|hardware" >&2
        exit 2
        ;;
esac

sudo apt update
sudo apt install -y --no-install-recommends \
    git tmux build-essential cmake pkg-config \
    libeigen3-dev libyaml-cpp-dev \
    python3-venv python3-vcs2l python3-yaml

# Numerical plotting packages are not needed on the companion computer.
if [[ "${profile}" == "sitl" ]]; then
    sudo apt install -y --no-install-recommends \
        python3-numpy python3-matplotlib python3-scipy
fi

echo "PX4 ${profile} build/runtime tools ready."
