"""
Neural Network Training Script for Mac
Save as: ~/wall_follower_nn/scripts/train_nn.py
"""

import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import seaborn as sns
import pickle
import os
from pathlib import Path
import json
from datetime import datetime

PROJECT_ROOT = Path().resolve() # current working directory  , REPLACED Path.home() WITH PROJECT_ROOT

# Set style for better plots
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (12, 6)

class WallFollowerDataset(Dataset):
    """Dataset for wall following behavior"""
    def __init__(self, features, labels):
        self.features = torch.FloatTensor(features)
        self.labels = torch.FloatTensor(labels)
    
    def __len__(self):
        return len(self.features)
    
    def __getitem__(self, idx):
        return self.features[idx], self.labels[idx]

class WallFollowerNet(nn.Module):
    """Neural network for wall following control"""
    def __init__(self, input_size=4, hidden_sizes=[64, 32], output_size=2, dropout=0.2):
        super(WallFollowerNet, self).__init__()
        
        layers = []
        prev_size = input_size
        
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(prev_size, hidden_size))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(dropout))
            prev_size = hidden_size
        
        layers.append(nn.Linear(prev_size, output_size))
        
        self.network = nn.Sequential(*layers)
    
    def forward(self, x):
        return self.network(x)

def load_and_preprocess_data(csv_path, remove_safety=True):
    """Load CSV and prepare features/labels"""
    print(f"📂 Loading data from {csv_path}...")
    
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")
    
    df = pd.read_csv(csv_path)
    print(f"   Total samples loaded: {len(df)}")
    
    # Data quality check
    print("\n📊 Data Quality Report:")
    print(f"   Columns: {list(df.columns)}")
    print(f"   Shape: {df.shape}")
    print(f"   Missing values: {df.isnull().sum().sum()}")
    
    # Remove safety events if requested
    if remove_safety and 'safety_flag' in df.columns:
        df_clean = df[df['safety_flag'] == 0].copy()
        print(f"   Removed safety events: {len(df) - len(df_clean)} samples")
        print(f"   Clean samples: {len(df_clean)}")
    else:
        df_clean = df.copy()
    
    # Check if we have enough data
    if len(df_clean) < 100:
        raise ValueError(f"Insufficient data: only {len(df_clean)} samples. Collect more data!")
    
    # Define features and labels
    feature_cols = ['cte', 'theta', 'linear_speed', 'angular_speed']
    label_cols = ['speed_cmd', 'angular_cmd']
    
    # Verify columns exist
    for col in feature_cols + label_cols:
        if col not in df_clean.columns:
            raise ValueError(f"Required column '{col}' not found in CSV!")
    
    features = df_clean[feature_cols].values
    labels = df_clean[label_cols].values
    
    # Remove any NaN or inf values
    mask = np.isfinite(features).all(axis=1) & np.isfinite(labels).all(axis=1)
    features = features[mask]
    labels = labels[mask]
    
    print(f"   Final dataset size: {len(features)} samples")
    
    # Display statistics
    print("\n📈 Feature Statistics:")
    feature_df = pd.DataFrame(features, columns=feature_cols)
    print(feature_df.describe())
    
    print("\n🎯 Label Statistics:")
    label_df = pd.DataFrame(labels, columns=label_cols)
    print(label_df.describe())
    
    return features, labels, feature_cols, label_cols

def visualize_data(features, labels, feature_cols, label_cols, save_dir):
    """Create comprehensive data visualizations"""
    print("\n📊 Creating data visualizations...")
    
    # Create feature plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.ravel()
    
    for i, col in enumerate(feature_cols):
        axes[i].hist(features[:, i], bins=50, alpha=0.7, edgecolor='black')
        axes[i].set_xlabel(col)
        axes[i].set_ylabel('Frequency')
        axes[i].set_title(f'Distribution of {col}')
        axes[i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'feature_distributions.png'), dpi=150)
    print(f"   ✅ Saved: feature_distributions.png")
    
    # Create label plots
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    for i, col in enumerate(label_cols):
        axes[i].hist(labels[:, i], bins=50, alpha=0.7, edgecolor='black', color='orange')
        axes[i].set_xlabel(col)
        axes[i].set_ylabel('Frequency')
        axes[i].set_title(f'Distribution of {col}')
        axes[i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'label_distributions.png'), dpi=150)
    print(f"   ✅ Saved: label_distributions.png")
    
    # Correlation plot
    plt.figure(figsize=(10, 8))
    all_data = np.concatenate([features, labels], axis=1)
    all_cols = feature_cols + label_cols
    corr_df = pd.DataFrame(all_data, columns=all_cols)
    sns.heatmap(corr_df.corr(), annot=True, fmt='.2f', cmap='coolwarm', center=0)
    plt.title('Feature-Label Correlation Matrix')
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'correlation_matrix.png'), dpi=150)
    print(f"   ✅ Saved: correlation_matrix.png")
    
    plt.close('all')

def train_model(csv_path, epochs=100, batch_size=32, learning_rate=0.001, 
                hidden_sizes=[64, 32], test_split=0.2):
    """Train the neural network"""
    
    # Create results directory with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = os.path.join(PROJECT_ROOT, 'wall_follower_nn', 'results', timestamp)
    os.makedirs(results_dir, exist_ok=True)
    print(f"\n📁 Results directory: {results_dir}")
    
    # Load data
    features, labels, feature_cols, label_cols = load_and_preprocess_data(csv_path)
    
    # Visualize data
    visualize_data(features, labels, feature_cols, label_cols, results_dir)
    
    # Split data
    print("\n🔪 Splitting data...")
    X_train, X_test, y_train, y_test = train_test_split(
        features, labels, test_size=test_split, random_state=42, shuffle=True
    )
    print(f"   Train samples: {len(X_train)}")
    print(f"   Test samples: {len(X_test)}")
    
    # Normalize features
    print("\n⚙️  Normalizing features...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)
    
    # Save scaler
    models_dir = os.path.join(PROJECT_ROOT, 'wall_follower_nn', 'models')
    os.makedirs(models_dir, exist_ok=True)
    scaler_path = os.path.join(models_dir, 'scaler.pkl')
    with open(scaler_path, 'wb') as f:
        pickle.dump(scaler, f)
    print(f"   ✅ Scaler saved: {scaler_path}")
    
    # Create datasets
    train_dataset = WallFollowerDataset(X_train_scaled, y_train)
    test_dataset = WallFollowerDataset(X_test_scaled, y_test)
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=batch_size)
    
    # Initialize model
    device = torch.device('mps' if torch.backends.mps.is_available() else 'cpu')
    print(f"\n🖥️  Using device: {device}")
    
    model = WallFollowerNet(
        input_size=len(feature_cols), 
        hidden_sizes=hidden_sizes, 
        output_size=len(label_cols)
    )
    model.to(device)
    
    # Print model architecture
    print("\n🏗️  Model Architecture:")
    print(model)
    total_params = sum(p.numel() for p in model.parameters())
    print(f"   Total parameters: {total_params:,}")
    
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10
    )
    
    # Training loop
    train_losses = []
    test_losses = []
    best_test_loss = float('inf')
    
    print(f"\n🚀 Starting training for {epochs} epochs...")
    print("=" * 70)
    
    for epoch in range(epochs):
        # Training phase
        model.train()
        train_loss = 0.0
        
        for batch_features, batch_labels in train_loader:
            batch_features = batch_features.to(device)
            batch_labels = batch_labels.to(device)
            
            optimizer.zero_grad()
            outputs = model(batch_features)
            loss = criterion(outputs, batch_labels)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
        
        train_loss /= len(train_loader)
        train_losses.append(train_loss)
        
        # Validation phase
        model.eval()
        test_loss = 0.0
        with torch.no_grad():
            for batch_features, batch_labels in test_loader:
                batch_features = batch_features.to(device)
                batch_labels = batch_labels.to(device)
                outputs = model(batch_features)
                loss = criterion(outputs, batch_labels)
                test_loss += loss.item()
        
        test_loss /= len(test_loader)
        test_losses.append(test_loss)
        
        # Learning rate scheduling
        scheduler.step(test_loss)
        
        # Save best model
        if test_loss < best_test_loss:
            best_test_loss = test_loss
            best_model_path = os.path.join(models_dir, 'wall_follower_model.pth')
            torch.save(model.state_dict(), best_model_path)
        
        # Print progress
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f'Epoch [{epoch+1:3d}/{epochs}] | '
                  f'Train Loss: {train_loss:.6f} | '
                  f'Test Loss: {test_loss:.6f} | '
                  f'Best: {best_test_loss:.6f}')
    
    print("=" * 70)
    print(f"\n✅ Training complete!")
    print(f"   Best test loss: {best_test_loss:.6f}")
    print(f"   Model saved: {best_model_path}")
    
    # Plot training curves
    plt.figure(figsize=(12, 5))
    
    plt.subplot(1, 2, 1)
    plt.plot(train_losses, label='Train Loss', linewidth=2)
    plt.plot(test_losses, label='Test Loss', linewidth=2)
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss')
    plt.title('Training Progress')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(1, 2, 2)
    plt.plot(train_losses, label='Train Loss', linewidth=2)
    plt.plot(test_losses, label='Test Loss', linewidth=2)
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss (log scale)')
    plt.yscale('log')
    plt.title('Training Progress (Log Scale)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    curve_path = os.path.join(results_dir, 'training_curve.png')
    plt.savefig(curve_path, dpi=150)
    print(f"   ✅ Training curve saved: {curve_path}")
    
    # Evaluate on test set
    evaluate_model(model, test_loader, device, y_test, scaler, results_dir)
    
    # Save training config
    config = {
        'timestamp': timestamp,
        'epochs': epochs,
        'batch_size': batch_size,
        'learning_rate': learning_rate,
        'hidden_sizes': hidden_sizes,
        'test_split': test_split,
        'train_samples': len(X_train),
        'test_samples': len(X_test),
        'best_test_loss': best_test_loss,
        'feature_cols': feature_cols,
        'label_cols': label_cols,
        'device': str(device)
    }
    
    config_path = os.path.join(results_dir, 'training_config.json')
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"   ✅ Config saved: {config_path}")
    
    return model, scaler

def evaluate_model(model, test_loader, device, y_test, scaler, results_dir):
    """Evaluate model and create visualization"""
    print("\n📊 Evaluating model on test set...")
    
    model.eval()
    all_predictions = []
    all_labels = []
    
    with torch.no_grad():
        for batch_features, batch_labels in test_loader:
            batch_features = batch_features.to(device)
            outputs = model(batch_features)
            all_predictions.append(outputs.cpu().numpy())
            all_labels.append(batch_labels.numpy())
    
    predictions = np.vstack(all_predictions)
    labels = np.vstack(all_labels)
    
    # Calculate metrics
    mse = np.mean((predictions - labels) ** 2, axis=0)
    mae = np.mean(np.abs(predictions - labels), axis=0)
    
    print(f"\n📈 Test Set Metrics:")
    print(f"   Speed Command  - MSE: {mse[0]:.6f}, MAE: {mae[0]:.6f}")
    print(f"   Angular Command - MSE: {mse[1]:.6f}, MAE: {mae[1]:.6f}")
    
    # Plot predictions vs actual
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Speed command
    axes[0, 0].scatter(labels[:, 0], predictions[:, 0], alpha=0.3, s=10)
    axes[0, 0].plot([labels[:, 0].min(), labels[:, 0].max()], 
                     [labels[:, 0].min(), labels[:, 0].max()], 
                     'r--', linewidth=2, label='Perfect Prediction')
    axes[0, 0].set_xlabel('Actual Speed Command')
    axes[0, 0].set_ylabel('Predicted Speed Command')
    axes[0, 0].set_title('Speed Command: Predicted vs Actual')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Angular command
    axes[0, 1].scatter(labels[:, 1], predictions[:, 1], alpha=0.3, s=10)
    axes[0, 1].plot([labels[:, 1].min(), labels[:, 1].max()], 
                     [labels[:, 1].min(), labels[:, 1].max()], 
                     'r--', linewidth=2, label='Perfect Prediction')
    axes[0, 1].set_xlabel('Actual Angular Command')
    axes[0, 1].set_ylabel('Predicted Angular Command')
    axes[0, 1].set_title('Angular Command: Predicted vs Actual')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Time series comparison (first 500 samples)
    n_samples = min(500, len(labels))
    axes[1, 0].plot(labels[:n_samples, 0], label='Actual', alpha=0.7, linewidth=1.5)
    axes[1, 0].plot(predictions[:n_samples, 0], label='Predicted', alpha=0.7, linewidth=1.5)
    axes[1, 0].set_xlabel('Sample')
    axes[1, 0].set_ylabel('Speed Command')
    axes[1, 0].set_title('Speed Command Time Series')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(labels[:n_samples, 1], label='Actual', alpha=0.7, linewidth=1.5)
    axes[1, 1].plot(predictions[:n_samples, 1], label='Predicted', alpha=0.7, linewidth=1.5)
    axes[1, 1].set_xlabel('Sample')
    axes[1, 1].set_ylabel('Angular Command')
    axes[1, 1].set_title('Angular Command Time Series')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    eval_path = os.path.join(results_dir, 'evaluation_results.png')
    plt.savefig(eval_path, dpi=150)
    print(f"   ✅ Evaluation plot saved: {eval_path}")
    
    plt.close('all')

if __name__ == '__main__':
    # Configuration
    csv_path = os.path.join(PROJECT_ROOT, 'wall_follower_nn', 'data', 'pid.csv')
    
    print("=" * 70)
    print("🤖 WALL FOLLOWER NEURAL NETWORK TRAINING")
    print("=" * 70)
    
    try:
        # Train the model
        model, scaler = train_model(
            csv_path=csv_path,
            epochs=150,
            batch_size=64,
            learning_rate=0.001,
            hidden_sizes=[128, 64, 32],
            test_split=0.2
        )
        
        print("\n" + "=" * 70)
        print("🎉 TRAINING COMPLETE!")
        print("=" * 70)
        print("\n📝 Next Steps:")
        print("1. Review results in ~/wall_follower_nn/results/")
        print("2. Copy model to Docker: cd ~/wall_follower_nn && ./copy_model_to_docker.sh")
        print("3. Test in simulation!")
        
    except Exception as e:
        print(f"\n❌ Error during training: {e}")
        import traceback
        traceback.print_exc()