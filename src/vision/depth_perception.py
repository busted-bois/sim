import cv2
import numpy as np
from pathlib import Path
from src.vision.feed import VisionFrame

class DepthEstimator:
    """
    Handles depth estimation from RGB frames using a pre-trained MiDaS model.
    """
    
    # MiDaS v2.1 Small (ONNX) typical input size
    INPUT_SIZE = (256, 256)
    
    def __init__(self, model_path: str | Path | None = None):
        """
        Initializes the depth estimator with a MiDaS model.
        
        Args:
            model_path: Path to the .onnx model file. If None, it will look for 'models/midas_v21_small.onnx'.
        """
        if model_path is None:
            model_path = Path("models/midas_v21_small.onnx")
        else:
            model_path = Path(model_path)
            
        self.model_path = model_path
        self.net = None
        
        # Calibration parameters (placeholders, should be tuned for AirSim)
        # Formula: metric_depth = scale * relative_depth + offset
        # Note: MiDaS outputs inverse depth (disparity-like), so logic might vary.
        self.scale = 1.0
        self.offset = 0.0

    def load_model(self) -> bool:
        """Loads the ONNX model into OpenCV DNN."""
        if not self.model_path.exists():
            print(f"[depth] Model not found at {self.model_path}")
            return False
            
        try:
            self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
            # Set preferable backend and target to CPU (default)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            return True
        except Exception as e:
            print(f"[depth] Failed to load model: {e}")
            return False

    def estimate(self, frame: VisionFrame) -> np.ndarray | None:
        """
        Estimates relative depth from a VisionFrame.
        
        Returns:
            A 2D numpy array of relative depth values, or None if estimation fails.
        """
        if self.net is None:
            if not self.load_model():
                return None
                
        img = frame.image_rgb
        h, w = img.shape[:2]
        
        # MiDaS v2.1 Small preprocessing:
        # 1. Resize to 256x256
        # 2. Normalize to [0, 1]
        # 3. Mean subtraction and scaling (standard ImageNet-style or model-specific)
        blob = cv2.dnn.blobFromImage(
            img, 
            1.0 / 255.0, 
            self.INPUT_SIZE, 
            (123.675, 116.28, 103.53), 
            True, 
            False
        )
        
        self.net.setInput(blob)
        output = self.net.forward()
        
        # Output is typically (1, 256, 256)
        depth_map = output[0]
        
        # Resize back to original frame size
        depth_map = cv2.resize(depth_map, (w, h), interpolation=cv2.INTER_CUBIC)
        
        return depth_map

    def get_metric_depth(self, frame: VisionFrame) -> np.ndarray | None:
        """
        Estimates depth and applies calibration to get values in meters.
        """
        relative_depth = self.estimate(frame)
        if relative_depth is None:
            return None
            
        # MiDaS outputs are often inverse depth.
        # Simple linear calibration for now:
        metric_depth = (self.scale * relative_depth) + self.offset
        
        # Ensure no negative depths
        return np.maximum(metric_depth, 0.1)

    def calibrate(self, relative_map: np.ndarray, ground_truth_map: np.ndarray):
        """
        Calibrates the model's output using a ground truth depth map from AirSim.
        Uses simple linear regression to find scale and offset.
        """
        # Flatten and remove invalid/infinity values if any
        rel = relative_map.flatten()
        gt = ground_truth_map.flatten()
        
        # Basic linear fit: GT = scale * REL + offset
        A = np.vstack([rel, np.ones(len(rel))]).T
        self.scale, self.offset = np.linalg.lstsq(A, gt, rcond=None)[0]
        print(f"[depth] Calibrated: scale={self.scale:.4f}, offset={self.offset:.4f}")
