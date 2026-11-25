
import math
import time

import cosysairsim as airsim

#All are constants meant to be replaced with real values
CRUISE_ALT_M = 10.0          
H = (0.0, 0.0)               # home, taken as the spawn point, return back here
D = (30.0, 0.0)              # drop zone, 30 m north of home point
CENTERED_RADIUS = 1.0        # metres from the waypoint center to be classified as "centered" 
CRUISE_SPEED = 3.0           
DESCENT_SPEED = 0.7          


def write(msg):
    print(f"[CALL-OUT] {msg}", flush=True)



class AirSimVehicle:
    def __init__(self):
        """Connect to the running simulator and allow API control"""
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def arm(self):
        """Arms the motors so drone can takeoff"""
        self.client.armDisarm(True)

    def disarm(self):
        """Disarms the motors"""
        self.client.armDisarm(False)

    def position(self):
        """Get the drone's current position"""
        position = self.client.getMultirotorState().kinematics_estimated.position
        return position.x_val, position.y_val, -position.z_val        

    def takeoff(self, altitude_meters):
        """Takeoff vertically and go to the target altitude"""
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-altitude_meters, DESCENT_SPEED * 3).join()   

    def goto(self, x, y, altitude_meters):
        """Moves the drone to a specific location x, y, z and wait"""
        self.client.moveToPositionAsync(x, y, -altitude_meters, CRUISE_SPEED).join()

    def descend_to(self, x, y, altitude_meters):
        """Descend slowly vertically while horizontal remains consistent"""
        self.client.moveToPositionAsync(x, y, -altitude_meters, DESCENT_SPEED).join()

    def land(self):
        """Land at current position and wait"""
        self.client.landAsync().join()

    def landed(self):
        """Check landed state by checking altitude is less than 0.3 and speed is also less than 0.1"""
        state = self.client.getMultirotorState()
        altitude = -state.kinematics_estimated.position.z_val
        velocity = state.kinematics_estimated.linear_velocity
        speed = math.sqrt(velocity.x_val**2 + velocity.y_val**2 + velocity.z_val**2)
        return altitude < 0.3 and speed < 0.1



def centered(vehicle, waypoint):
    """Checks whether the drone is horizontally within range of waypoint"""
    x, y, _ = vehicle.position()
    return math.hypot(x - waypoint[0], y - waypoint[1]) <= CENTERED_RADIUS


def land_at(vehicle, waypoint, destination_name):
    """Fly to waypoint, recenter if necessary, descend at waypoint, and land, finished when in landed state."""
    write(f"Fly to {destination_name} at {CRUISE_ALT_M} m")
    vehicle.goto(waypoint[0], waypoint[1], CRUISE_ALT_M)
    time.sleep(2)                                   

    
    if not centered(vehicle, waypoint):
        write(f"Re-centering over {destination_name}")
        vehicle.goto(waypoint[0], waypoint[1], CRUISE_ALT_M)

    write(f"Descending at {destination_name}")
    vehicle.descend_to(waypoint[0], waypoint[1], 2.0)      # 2.0 is a placeholder for changing point between descension and landing          
    write(f"Landing at {destination_name}")
    vehicle.land()
    t0 = time.time()
    while not vehicle.landed() and time.time() - t0 < 30:   #30 second check for landing phase to prevent timingout
        time.sleep(0.2)

    ok = centered(vehicle, waypoint)
    write(f"Touched down at {destination_name}, centered={ok}")
    return ok


def fm1(v):
    """Main FM-1 Mission Logic"""
    write("Arming")
    v.arm()

    write("Takeoff")
    v.takeoff(CRUISE_ALT_M)
    land_at(v, D, "D")

    write("Takeoff again")
    v.takeoff(CRUISE_ALT_M)
    land_at(v, H, "H")

    write("Disarming")
    v.disarm()
    write("FM-1 complete")


if __name__ == "__main__":
    fm1(AirSimVehicle())