from src.control.algorithms import Algorithm, register

DIRECTIONS = [
    ("+X", 2.0, 0.0, 0.0),
    ("-X", -2.0, 0.0, 0.0),
    ("+Y", 0.0, 2.0, 0.0),
    ("-Y", 0.0, -2.0, 0.0),
    ("+Z", 0.0, 0.0, 2.0),
    ("-Z", 0.0, 0.0, -2.0),
]
DURATION_S = 5.0


@register("six_directions")
class SixDirections(Algorithm):
    def run(self, client):
        client.takeoffAsync().join()

        for label, vx, vy, vz in DIRECTIONS:
            print(f"[six_directions] Moving {label}")
            client.moveByVelocityAsync(vx, vy, vz, DURATION_S).join()

        print("[six_directions] Complete — hovering")
        client.hoverAsync().join()
