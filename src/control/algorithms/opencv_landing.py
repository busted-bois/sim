from src.control.algorithms import Algorithm, register
from src.control.flight_client import FlightClient
from src.control.primitives import rotate_yaw, takeoff_with_settle
from src.vision.processing import find_large_grey_wall


@register("opencv_landing")
class OpenCvLanding(Algorithm):
    config_section = None

    def run(self, client: FlightClient):
        """
        This algorithm flies forward until it detects a large grey wall,
        then it moves backward and lands.
        """
        print("[opencv_landing] Algorithm started.")

        # Takeoff
        takeoff_with_settle(client, max_attempts=1, label="opencv_landing")
        print("[opencv_landing] Takeoff complete.")

        # Rotate 180 degrees
        print("[opencv_landing] Rotating 180 degrees...")
        rot_cfg = self._config.get("startup_rotation", {})
        rate_dps = float(rot_cfg.get("rate_dps", 60))
        duration_s = float(rot_cfg.get("duration_s", 3.0))
        rotate_yaw(client, rate_dps, duration_s, label="opencv_landing")
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

