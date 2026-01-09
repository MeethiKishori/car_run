# Wall Follower Neural Network Training

## Directory Structure
- `data/` - CSV files from ROS2 simulation
- `models/` - Trained model files (.pth)
- `notebooks/` - Jupyter notebooks for analysis
- `results/` - Training results and visualizations
- `scripts/` - Training and utility scripts

## Quick Start
1. Activate environment: `source venv/bin/activate` # basically activate ros2 env
2. Copy CSV from Docker: `./copy_data_from_docker.sh`
3. Train model: `python scripts/train_nn.py`
4. Copy model to Docker: `./copy_model_to_docker.sh`

## Requirements
- Python 3.8+
- PyTorch
- pandas, numpy, scikit-learn, matplotlib
