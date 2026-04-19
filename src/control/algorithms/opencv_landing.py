from src.control.algorithms import Algorithm, register
from src.vision.processing import find_large_grey_wall


@register("opencv_landing")
class OpenCvLanding(Algorithm):
    def run(self, client):
        """
        This algorithm flies forward until it detects a large grey wall,
        then it moves backward and lands.
        """
        print("[opencv_landing] Algorithm started.")

        # Takeoff
        client.takeoffAsync().join()
        print("[opencv_landing] Takeoff complete.")

        # Rotate 180 degrees
        print("[opencv_landing] Rotating 180 degrees...")
        client.rotateByYawRateAsync(60, 3).join()
        print("[opencv_landing] Rotation complete.")

        # 1. Search for the wall
        print("[opencv_landing] Searching for wall...")
        found_target = False
        client.moveByVelocityAsync(2, 0, 0, 999)  # Move forward indefinitely
        while not found_target:
            frame = self.latest_frame()
            if frame:
                if find_large_grey_wall(frame):
                    print("[opencv_landing] Wall detected.")
                    found_target = True
                    # Stop and move backward
                    print("[opencv_landing] Target seen, moving back...")
                    client.moveByVelocityAsync(-2, 0, 0, 2).join()
                else:
                    # If no wall is found, continue moving forward
                    print("[opencv_landing] No wall found, continuing forward...")
            else:
                print("[opencv_landing] No vision frame available.")

        # 2. Land
        print("[opencv_landing] Landing...")
        client.landAsync().join()

        print("[opencv_landing] Algorithm finished.")

