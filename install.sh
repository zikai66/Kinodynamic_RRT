set -e

echo "🔧 Updating system packages..."
sudo apt-get update -y
sudo apt-get install -y python3 python3-venv python3-tk build-essential

if [ ! -d ".venv" ]; then
  echo "Creating virtual environment..."
  python3 -m venv .venv
fi

echo "Activating virtual environment..."
source .venv/bin/activate

echo "Upgrading pip inside venv..."
python -m pip install --upgrade pip setuptools wheel

echo "Installing dependencies..."
pip install numpy scipy matplotlib pybullet pybullet-planning
