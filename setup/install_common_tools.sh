#!/usr/bin/env bash
# Install compilation and source-management tools common to all native scopes.
# SITL additionally includes plotting/numerical analysis packages; the common
# and hardware profiles intentionally use the same lean dependency set.
set -euo pipefail

profile="${1:-sitl}"
case "${profile}" in
    sitl|hardware|common) ;;
    *) echo "Usage: ./setup/install_common_tools.sh [common|sitl|hardware]" >&2; exit 2 ;;
esac

echo "Installing ${profile} toolkit build and runtime utilities..."

# Dependencies of the maintained ROS 2 C++ packages and source manager.
sudo apt update
sudo apt install -y --no-install-recommends \
    git tmux build-essential cmake pkg-config \
    libeigen3-dev libyaml-cpp-dev \
    python3-venv python3-vcs2l python3-yaml

# Plotting/numerical analysis is part of the SITL developer image, not a
# prerequisite of the companion-computer runtime.
if [[ "${profile}" == "sitl" ]]; then
    sudo apt install -y --no-install-recommends \
        python3-numpy python3-matplotlib python3-scipy
fi

echo "Toolkit ${profile} tools ready."
