from src.control.algorithms import Algorithm, register
from src.vision.processing import find_red_circles


@register("opencv_landing")
class OpenCvLanding(Algorithm):
    def run(self, client):
        """
        This algorithm is a placeholder for a real implementation.
        It demonstrates how to use the vision processing module
        to find a target and how one might implement a landing sequence.
        """
        print("[opencv_landing] Algorithm started.")

        # Takeoff
        client.takeoffAsync().join()
        print("[opencv_landing] Takeoff complete.")

        # 1. Search for the landing pad
        print("[opencv_landing] Searching for landing pad...")
        found_target = False
        client.moveByVelocityAsync(2, 0, 0, 999) # Move forward indefinitely
        while not found_target:
            frame = self.latest_frame()
            if frame:
                circles = find_red_circles(frame)
                if circles:
                    print(f"[opencv_landing] Found {len(circles)} red circle(s).")
                    # For simplicity, we'll just take the first one
                    target_x, target_y, radius = circles[0]
                    print(f"[opencv_landing] Target at ({target_x}, {target_y}) with radius {radius}.")
                    found_target = True
                    # Stop and move backward
                    print("[opencv_landing] Target seen, moving back...")
                    client.moveByVelocityAsync(-2, 0, 0, 2).join()
                else:
                    # If no target is found, continue moving forward
                    print("[opencv_landing] No target found, continuing forward...")
            else:
                print("[opencv_landing] No vision frame available.")

        # 2. Land
        print("[opencv_landing] Landing...")
        client.landAsync().join()

        print("[opencv_landing] Algorithm finished.")
