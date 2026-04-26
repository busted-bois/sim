import urllib.request
from pathlib import Path

def download_midas():
    model_url = "https://github.com/intel-isl/MiDaS/releases/download/v2_1/model-small.onnx"
    models_dir = Path("models")
    models_dir.mkdir(exist_ok=True)
    
    target_path = models_dir / "midas_v21_small.onnx"
    
    if target_path.exists():
        print(f"Model already exists at {target_path}")
        return

    print(f"Downloading MiDaS v2.1 Small from {model_url}...")
    try:
        urllib.request.urlretrieve(model_url, target_path)
        print(f"Successfully downloaded to {target_path}")
    except Exception as e:
        print(f"Failed to download model: {e}")

if __name__ == "__main__":
    download_midas()
