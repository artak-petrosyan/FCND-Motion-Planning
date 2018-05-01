import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils_graph_imp import a_star, heuristic, closest_point, prune_path, create_grid_and_edges, create_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from io import StringIO

import matplotlib.pyplot as plt


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_glbal_position_param):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # Goal global position
        self._goal_global_position = goal_glbal_position_param

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    #Goal global position public getter setter methods
    #Public property for goal global position (getter)
    @property
    def goal_global_position(self):
        return self._goal_global_position


    #set method for goal global position
    def set_goal_global_position(self, longitude, latitude, altitude):
        """Set the drone's goal global position to these coordinates"""
        self._goal_global_position = np.array([latitude, longitude, altitude])


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
        SAFETY_DISTANCE = 3

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0 = None
        lon0 = None
        with open('colliders.csv') as f:
            #Read first line
            line=f.readline()
            #Split line after replacing latitude / longitude delimiter comma to whitespace for numpy 
            home_coord=np.loadtxt(StringIO(line.replace(', ', ' ')), delimiter=' ', usecols=(1, 3), unpack=True)
            lat0 = home_coord[0]
            lon0 = home_coord[1]
        print('Coordinates from the first line of the "colliders.csv" lat0 = {0},  lon0 ={1}'.format(lat0, lon0))
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)
        print('Global home position lon = {0}, lat = {1}, alt = {2}'.format(self.global_home[0], self.global_home[1], self.global_home[2]))

        # TODO: retrieve current global position
        global_pos = (self._longitude, self._latitude, self._altitude)
        print('Global current position lon = {0}, lat = {1}, alt = {2}'.format(global_pos[0], global_pos[1], global_pos[2]))
 
        # TODO: convert to current local position using global_to_local()
        local_pos = global_to_local(global_pos, self.global_home)
        print('Local current position lon = {0}, lat = {1}, alt = {2}'.format(local_pos[0], local_pos[1], local_pos[2]))
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        #print(edges)
        graph = create_graph(edges)
        print(graph)
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        

        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.ceil(local_pos[0] - north_offset)), int(np.ceil(local_pos[1] - east_offset)))
        graph_start = closest_point(graph, grid_start)
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        local_goal = global_to_local(self.goal_global_position, self.global_home)
        grid_goal = (int(np.ceil(local_goal[0] - north_offset)), int(np.ceil(local_goal[1] - east_offset)))
        graph_goal = closest_point(graph, grid_goal)
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Graph Start and Goal: ', graph_start, graph_goal)
        
        path, _ = a_star(graph, heuristic, graph_start, graph_goal)
        path.append(grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        path  = prune_path(path)
        print(path)
        print('=====================================================')
        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]
        print(waypoints)
        print('=====================================================')
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
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
    parser.add_argument('-g', '--goal', nargs='*', type=float,  default=[37.796751, -122.401289, 0.0], help="Goal global coordinates (longitude, latitude, altitude), i.e. '37.796751 -122.401289 0.0'")
    args = parser.parse_args()
    print("Goal global position lon={0} lat={1} alt={2}".format(args.goal[0], args.goal[1], args.goal[2]))
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, (args.goal[0], args.goal[1], args.goal[2]))
    time.sleep(1)

    drone.start()
