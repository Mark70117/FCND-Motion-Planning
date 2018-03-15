#import sys
#sys.path.insert(0,'../MRA/udacidrone')

import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from pruning_utils import prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

COLLIDERS_FN = 'colliders.csv'

# rubric key
# IYPPA-1 -- Implementing Your Path Planning Algorithm // set home position
# IYPPA-2 -- retrieve current position in geodetic coordinates
# IYPPA-3 -- change start point to current local position
# IYPPA-4 -- change goal point
# IYPPA-5 -- write search algorithm
# IYPPA-6 -- cull waypoints


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # IYPPA-1 begin
        # DONE: read lat0, lon0 from colliders into floating point values
        with open(COLLIDERS_FN) as f:
            latLonStrArr = f.readline().rstrip().replace('lat0','').replace('lon0 ','').split(',')
            lat0 = float(latLonStrArr[0])
            lon0 = float(latLonStrArr[1])

        # DONE : set home position to (lat0, lon0, 0)
        # DONE : CORRECT set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        # IYPPA-1 end


        # IYPPA-2 begin
        # DONE: retrieve current global position
        # print (self.global_home)
        global_position = [self._longitude, self._latitude, self._altitude]
        # print(global_position)

        # DONE: convert to current local position using global_to_local()
        current_local_pos = global_to_local(global_position,self.global_home)
        # print (current_local_pos)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # IYPPA-2 end

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        
        # Define a grid for a particular altitude and safety margin around obstacles

        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)

        # IYPPA-3 begin
        # DONE: convert start position to current position rather than map center
        #start = (int(current_local_pos[0]+north_offset), int(current_local_pos[1]+east_offset))
        grid_start = (int(current_local_pos[0]-north_offset), int(current_local_pos[1]-east_offset))
        # IYPPA-3 end

        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        # IYPPA-4 start
        # DONE: adapt to set goal as latitude / longitude position and convert
        # 37.795023,-122.400325 fails
        # 37.793515,-122.397632 works
        # 37.793375,-122.398791 fails
        # 37.793490,-122.397895 works
        # 37.793600,-122.397927 works
        # 37.793528,-122.398560 crash
        # 37.793794,-122.396580 crash
        #goal_local_pos = global_to_local([-122.397927,37.793600,self.global_home[2]],self.global_home)
        goal_local_pos = global_to_local([-122.402034,37.797330,self.global_home[2]],self.global_home)
        grid_goal = (int(goal_local_pos[0]-north_offset), int(goal_local_pos[1]-east_offset))
        grid_goal = (461,510)
        grid_goal = (325,455)
        goal_local_pos = global_to_local([-122.396420,37.793691,self.global_home[2]],self.global_home)
        grid_goal = (int(goal_local_pos[0]-north_offset), int(goal_local_pos[1]-east_offset))
        goal_local_pos = global_to_local([-122.402224,37.797330,self.global_home[2]],self.global_home)
        grid_goal = (int(goal_local_pos[0]-north_offset), int(goal_local_pos[1]-east_offset))
        print ("fake grid_goal", grid_goal)
        grid_goal = (918, 21) #XYZZY
        grid_goal = (795, 117) #XYZZY
        #grid_goal = (851, 21) #GOAL in rubric
        # IYPPA-4 end

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # IYPPA-5 in a_star [file: planning_utils.py]
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        
        # DONE: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # IYPPA-6 begin
        print ("path: ", path)
        path = prune_path(path)
        print ("revised path: ", path)
        # IYPPA-6 end
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
