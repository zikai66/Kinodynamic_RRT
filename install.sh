set -e

echo "Updating system packages..."
sudo apt-get update -y
sudo apt-get install -y python3 python3-pip python3-tk build-essential

echo "Upgrading pip..."
python3 -m pip install --upgrade pip setuptools wheel

echo "Installing Python dependencies..."
pip install numpy scipy matplotlib pybullet


echo "Installation complete!"
echo "To run the demo, execute:"
echo "  python3 demo.py"
