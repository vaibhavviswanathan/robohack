#!/usr/bin/env bash
set -euo pipefail

# ---- Config you might tweak ----
REPO_DIR="/root/repos"
OPENPI_DIR="$REPO_DIR/openpi"
export OPENPI_DATA_HOME="${OPENPI_DATA_HOME:-/root/openpi_data}"
export GIT_LFS_SKIP_SMUDGE=1
export XLA_PYTHON_CLIENT_MEM_FRACTION="${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.9}"
export TOKENIZERS_PARALLELISM=false

echo "[1/7] System deps"
apt-get update
apt-get install -y git-lfs curl
git lfs install

echo "[2/7] Install uv"
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
# Make sure uv is on PATH for non-interactive shells
export PATH="$HOME/.cargo/bin:$PATH"
uv --version

echo "[3/7] Clone openpi"
mkdir -p "$REPO_DIR"
cd "$REPO_DIR"
if [ ! -d "$OPENPI_DIR" ]; then
  git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git
else
  cd "$OPENPI_DIR"
  git pull
  git submodule update --init --recursive
fi

echo "[4/7] Create env + install"
cd "$OPENPI_DIR"
uv sync
uv pip install -e .

echo "[5/7] Create data dirs"
mkdir -p "$OPENPI_DATA_HOME"

echo "[6/7] Compute norm stats (pi0.5 libero)"
uv run scripts/compute_norm_stats.py --config-name pi05_libero

echo "[7/7] Kick off training"
uv run scripts/train.py pi05_libero --exp-name=vessl_demo --overwrite

