import airsim
import cv2
import numpy as np
import time
import sys
from src.vision.feed import VisionFrame
from src.vision.depth_perception import DepthEstimator
from src.config import load_config, simulator_endpoint

def main():
    """
    Automated calibration tool for depth estimation.
    Compares MiDaS relative output against AirSim ground truth metric depth.
    """
    print("Loading config and connecting to AirSim...")
    try:
        config = load_config()
    except Exception as e:
        print(f"[calibrate] Error: Could not load config: {e}")
        return

    host, port = simulator_endpoint(config)
    print(f"[calibrate] Targeting AirSim at {host}:{port}...")

    # Use a shorter timeout for calibration tool so it fails fast if sim is not running
    try:
        client = airsim.MultirotorClient(ip=host, port=port, timeout_value=5)
        client.confirmConnection()
    except Exception as e:
        print(f"\n[calibrate] Error: Could not connect to AirSim at {host}:{port}")
        print(f"[calibrate] Details: {e}")
        print("[calibrate] Hint: Ensure the simulator is running and PROJECT_PATH is correct.")
        return
    
    # Try to load existing calibration from config
    depth_cfg = config.get("vision", {}).get("depth", {})
    model_path = depth_cfg.get("model_path")
    
    estimator = DepthEstimator(model_path)
    if not estimator.load_model():
        print(f"[calibrate] Error: Could not load MiDaS model from {estimator.model_path}")
        print("[calibrate] Hint: Run 'python scripts/download_models.py' first.")
        return

    print("\n" + "="*50)
    print("          DEPTH CALIBRATION TOOL")
    print("="*50)
    print("INSTRUCTIONS:")
    print("1. Ensure AirSim is running and the drone is spawned.")
    print("2. FLY THE DRONE manually while this script runs.")
    print("3. You must be between 0.5m and 50m from objects for data to be valid.")
    print("4. Avoid looking directly at the sky or sitting on the ground.")
    print("="*50 + "\n")

    all_rel = []
    all_gt = []
    
    samples_to_collect = 1000
    collected = 0
    start_time = time.time()
    
    try:
        while collected < samples_to_collect:
            # Request RGB Scene and Ground Truth Depth (Planar)
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
                airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True)
            ])
            
            if not responses or len(responses) < 2:
                print("[calibrate] Warning: Received no images from AirSim", end="\r")
                continue
                
            # Process RGB
            response_rgb = responses[0]
            if response_rgb.width == 0 or response_rgb.height == 0:
                print("[calibrate] Warning: Empty RGB image", end="\r")
                continue
                
            img1d = np.frombuffer(response_rgb.image_data_uint8, dtype=np.uint8)
            img_bgr = img1d.reshape(response_rgb.height, response_rgb.width, 3)
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            
            frame = VisionFrame(
                seq=collected, 
                timestamp_s=time.time(), 
                frame_age_s=0, 
                width=response_rgb.width, 
                height=response_rgb.height, 
                image_rgb=img_rgb
            )
            
            # Process Ground Truth Depth
            response_gt = responses[1]
            gt_map = np.array(response_gt.image_data_float, dtype=np.float32)
            gt_map = gt_map.reshape(response_gt.height, response_gt.width)
            
            # Get Model Relative Estimate
            rel_map = estimator.estimate(frame)
            
            if rel_map is not None:
                # Masking: Only use pixels within a reliable range (0.5m to 50m)
                mask = (gt_map > 0.5) & (gt_map < 50.0)
                
                # Further mask: only use center of frame
                h, w = gt_map.shape
                center_mask = np.zeros_like(mask, dtype=bool)
                center_mask[h//4:3*h//4, w//4:3*w//4] = True
                final_mask = mask & center_mask
                
                if np.any(final_mask):
                    all_rel.append(rel_map[final_mask])
                    all_gt.append(gt_map[final_mask])
                    collected += 1
                    # Progress update with flush to ensure it shows in terminal
                    sys.stdout.write(f"\r[calibrate] Progress: {collected}/{samples_to_collect} samples collected...")
                    sys.stdout.flush()
                else:
                    # Diagnostics for the user
                    min_gt = np.min(gt_map)
                    max_gt = np.max(gt_map)
                    sys.stdout.write(f"\r[calibrate] No valid pixels in frame (Min Dist: {min_gt:.1f}m, Max: {max_gt:.1f}m)    ")
                    sys.stdout.flush()
            
            time.sleep(0.05)

        print("\n\n[calibrate] Data collection complete. Analyzing...")
        rel_data = np.concatenate(all_rel)
        gt_data = np.concatenate(all_gt)
        
        # Linear Regression: GroundTruth = scale * Relative + offset
        A = np.vstack([rel_data, np.ones(len(rel_data))]).T
        # We use a robust solver
        solution = np.linalg.lstsq(A, gt_data, rcond=None)
        scale, offset = solution[0]
        
        # Calculate R-squared for confidence
        residuals = gt_data - (scale * rel_data + offset)
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((gt_data - np.mean(gt_data))**2)
        r_squared = 1 - (ss_res / ss_tot)

        print("\n" + "="*50)
        print("                RESULTS")
        print("="*50)
        print(f"Optimal Scale:  {scale:.6f}")
        print(f"Optimal Offset: {offset:.6f}")
        print(f"R-squared:      {r_squared:.4f}")
        print("-" * 50)
        print("Interpreting R-squared:")
        if r_squared > 0.9: print("  Excellent: Depth estimation is highly linear and accurate.")
        elif r_squared > 0.7: print("  Good: Usable for obstacle avoidance.")
        else: print("  Poor: Try collecting data with more variety in distances.")
        print("="*50)
        
        print("\nACTION: Copy these values to your sim.config.json:")
        print("under vision -> depth -> calibration:")
        print(f'  "scale": {scale:.6f},')
        print(f'  "offset": {offset:.6f}')
        print("="*50 + "\n")

    except KeyboardInterrupt:
        print("\n[calibrate] Interrupted by user.")
    except Exception as e:
        print(f"\n[calibrate] ERROR: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
