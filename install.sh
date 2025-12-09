set -euo pipefail

PYTHON_BIN="${PYTHON:-python3}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "[install] Error: python3 not found. Set \$PYTHON to your Python 3 path." >&2
  exit 1
fi

if [ ! -d ".venv" ]; then
  echo "[install] Creating virtual environment at .venv"
  "$PYTHON_BIN" -m venv .venv
fi
source .venv/bin/activate

python -m pip install --upgrade pip setuptools wheel

python -m pip install \
  numpy \
  matplotlib \
  pybullet \
  pybullet-planning

echo "[install] Done. To use: source .venv/bin/activate"
