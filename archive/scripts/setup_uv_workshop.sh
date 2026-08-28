#!/usr/bin/env bash
# mpc_dog workshop environment via uv (Python 3.11 + PyMPC sim deps)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYMPC="$ROOT/external/Quadruped-PyMPC"
ACADOS_DIR="$PYMPC/quadruped_pympc/acados"

export PATH="${HOME}/.local/bin:${PATH}"

if ! command -v uv >/dev/null 2>&1; then
  echo "Installing uv..."
  python3 -m pip install --user uv
  export PATH="${HOME}/.local/bin:${PATH}"
fi

echo "=== uv workshop setup ==="
cd "$ROOT"

if [[ ! -d "$PYMPC" ]]; then
  "$ROOT/scripts/setup_references.sh"
fi

if [[ ! -f "$ACADOS_DIR/CMakeLists.txt" ]]; then
  echo "ERROR: acados submodule missing. Run: ./scripts/setup_references.sh && git submodule update --init --recursive" >&2
  exit 1
fi

# Python 3.11 venv
uv python install 3.11
uv venv --python 3.11 .venv
# shellcheck disable=SC1091
source "$ROOT/.venv/bin/activate"

echo "Installing mpc_dog + workshop deps..."
uv sync --extra workshop

# acados build (reuse if present)
if [[ ! -f "$ACADOS_DIR/lib/libacados.so" ]]; then
  echo "Building acados..."
  mkdir -p "$ACADOS_DIR/build"
  cmake -S "$ACADOS_DIR" -B "$ACADOS_DIR/build" \
    -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_INSTALL_PREFIX="$ACADOS_DIR"
  cmake --build "$ACADOS_DIR/build" --target install -j"$(nproc)"
fi

uv pip install -e "$ACADOS_DIR/interfaces/acados_template"
uv pip install -e "$PYMPC"

# env activation helper
cat > "$ROOT/.env.workshop" <<EOF
# source: . .env.workshop
export ACADOS_SOURCE_DIR="$ACADOS_DIR"
export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:+\$LD_LIBRARY_PATH:}$ACADOS_DIR/lib"
export MUJOCO_GL=egl
export PATH="$ROOT/.venv/bin:\$PATH"
EOF

# ipykernel for notebooks
python -m ipykernel install --user --name mpc-dog-workshop --display-name "mpc-dog (uv workshop)"

echo ""
echo "=== uv workshop ready ==="
echo "  source .venv/bin/activate && . .env.workshop"
echo "  uv run python scripts/run_workshop_pipeline.py   # demos + executed notebooks"
