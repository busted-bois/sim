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
                else:
                    # If no target is found, fly in a square pattern
                    print("[opencv_landing] No target found, searching...")
                    client.moveByVelocityAsync(2, 0, 0, 2).join()  # Move forward
                    client.moveByVelocityAsync(0, 2, 0, 2).join()  # Move right
                    client.moveByVelocityAsync(-2, 0, 0, 2).join() # Move back
                    client.moveByVelocityAsync(0, -2, 0, 2).join() # Move left
            else:
                print("[opencv_landing] No vision frame available.")
                client.hoverAsync().join()

        # 2. Align with the landing pad
        # This is where you would implement PID controllers to align the
        # drone over the target based on the (target_x, target_y) coordinates.
        print("[opencv_landing] Aligning with landing pad (not implemented).")

        # 3. Descend and land
        # This is where you would control the throttle to descend
        # while keeping the drone aligned.
        print("[opencv_landing] Descending and landing (not implemented).")

        # For now, just hover and then land
        client.hoverAsync().join()
        client.landAsync().join()

        print("[opencv_landing] Algorithm finished.")
