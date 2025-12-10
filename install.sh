set -euo pipefail

echo "=== Updating system packages ==="
sudo apt-get update -y
sudo apt-get install -y python3 python3.11 python3.11-venv python3-tk build-essential

PYTHON_BIN=$(command -v python3.11 || command -v python3)

if [ ! -d ".venv" ]; then
  echo "=== Creating virtual environment (.venv) ==="
  "$PYTHON_BIN" -m venv .venv
fi

echo "=== Activating virtual environment ==="
source .venv/bin/activate

echo "=== Upgrading pip and build tools ==="
python -m pip install --upgrade pip setuptools wheel

echo "=== Installing Python dependencies ==="
pip install "numpy<2" "pybullet<4" matplotlib

echo
echo "Setup complete!"
echo "To activate the virtual environment later, run:"
echo "source .venv/bin/activate"
