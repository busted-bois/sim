import airsim
import cv2
import numpy as np
import time
from src.vision.feed import VisionFrame
from src.vision.depth_perception import DepthEstimator
from src.config import load_config

def main():
    """
    Automated calibration tool for depth estimation.
    Compares MiDaS relative output against AirSim ground truth metric depth.
    """
    config = load_config()
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    estimator = DepthEstimator()
    if not estimator.load_model():
        print("[calibrate] Error: Could not load MiDaS model. Did you run scripts/download_models.py?")
        return

    print("\n" + "="*50)
    print("          DEPTH CALIBRATION TOOL")
    print("="*50)
    print("INSTRUCTIONS:")
    print("1. Ensure AirSim is running.")
    print("2. The script will collect 100 samples.")
    print("3. FLY THE DRONE manually while the script runs.")
    print("4. Move towards walls, fly over ground, and approach objects.")
    print("="*50 + "\n")

    all_rel = []
    all_gt = []
    
    samples_to_collect = 100
    collected = 0
    
    try:
        while collected < samples_to_collect:
            # Request RGB Scene and Ground Truth Depth (Planar)
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
                airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True)
            ])
            
            if not responses or len(responses) < 2:
                continue
                
            # Process RGB
            response_rgb = responses[0]
            if response_rgb.width == 0 or response_rgb.height == 0:
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
                # This ignores the sky (inf) and very close clipping
                mask = (gt_map > 0.5) & (gt_map < 50.0)
                
                # Further mask: only use center of frame to avoid lens distortion effects
                h, w = gt_map.shape
                center_mask = np.zeros_like(mask, dtype=bool)
                center_mask[h//4:3*h//4, w//4:3*w//4] = True
                final_mask = mask & center_mask
                
                if np.any(final_mask):
                    all_rel.append(rel_map[final_mask])
                    all_gt.append(gt_map[final_mask])
                    collected += 1
                    print(f"[calibrate] Collected sample {collected}/{samples_to_collect}", end="\r")
            
            time.sleep(0.1) # Throttling to keep AirSim responsive

        if not all_rel:
            print("\n[calibrate] Failed: No valid data points collected.")
            return

        print("\n\n[calibrate] Calculating optimal regression...")
        rel_data = np.concatenate(all_rel)
        gt_data = np.concatenate(all_gt)
        
        # Linear Regression: GroundTruth = scale * Relative + offset
        # A * [scale, offset]^T = gt_data
        A = np.vstack([rel_data, np.ones(len(rel_data))]).T
        scale, offset = np.linalg.lstsq(A, gt_data, rcond=None)[0]
        
        # Calculate R-squared for confidence
        residuals = gt_data - (scale * rel_data + offset)
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((gt_data - np.mean(gt_data))**2)
        r_squared = 1 - (ss_res / ss_tot)

        print("\n" + "="*50)
        print("RESULTS")
        print("="*50)
        print(f"Optimal Scale:  {scale:.6f}")
        print(f"Optimal Offset: {offset:.6f}")
        print(f"R-squared:      {r_squared:.4f} (Closer to 1.0 is better)")
        print("="*50)
        print("\nACTION: Update your sim.config.json with these values:")
        print(f'  "calibration": {{ "scale": {scale:.6f}, "offset": {offset:.6f} }}')
        print("="*50 + "\n")

    except KeyboardInterrupt:
        print("\n[calibrate] Interrupted by user.")
    except Exception as e:
        print(f"\n[calibrate] Error: {e}")

if __name__ == "__main__":
    main()
