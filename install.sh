set -euo pipefail

echo "Updating system packages..."
sudo apt-get update -y
sudo apt-get install -y python3 python3-venv python3-tk build-essential

if [ ! -d ".venv" ]; then
  echo "Creating virtual environment..."
  python3 -m venv .venv
fi

source .venv/bin/activate

echo "Upgrading pip..."
python -m pip install --upgrade pip setuptools wheel

echo "Installing Python dependencies..."
pip install numpy matplotlib pybullet

echo "To activate the virtual environment, run:"
echo "source .venv/bin/activate"