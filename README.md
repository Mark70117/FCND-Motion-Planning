# FCND - 3D Motion Planning

This the _3D Motion Planning_ project for Udacity's Flying Car Nanodegree course.

Mark Anderson // Feb 2018 cohort

## Rubric items

### Explain the Starter Code [EtSC]

#### import

new
import argparse
import msgpack
from enum import auto
from planning_utils import a_star, heuristic, create_grid
from udacidrone.frame_utils import global_to_local

old
from udacidrone.connection import WebSocketConnection

#### States
use auto() to automatically uniquely enumerate states
add PLANNING state

#### init
rename all_waypoints to waypoints

#### local_position_callback
remove call to self.calculate_box before call to self.waypoint_transistion()
add print statement to print target, local
change references from self.all_waypoints to self.waypoints

#### velocity_callback
no differencece

####  state
deal with addition of PLANNING state
old
ARMING -{takeoff_transition}-> TAKEOFF
new
ARMING -{plan_path}-> PLANNING -{takeoff_transition}-> TAKEOFF

#### arming_transistion
move self.flight_state = States.ARMING from last thing executed to first thing excuted
move self.arm() before self.take_control()
remove call to self.set_home_position

#### takeoff_transition
move self.flight_state = States.TAKEOFF from last thing executed to first thing excuted
remove hardcoding of target altitude
assume self.target_position already set

####  waypoint_transition
move self.flight_state = States.WAYPOINT from last thing executed to first thing excuted
use heading parameter in self.cmd_position instead of hardcode to 0

#### landing_transition
move self.flight_state = States.LANDING from last thing executed to first thing excuted

#### disarming_transition
move self.flight_state = States.DISARMING from last thing executed to first thing excuted

#### manual_transition
move self.flight_state = States.MANUAL from last thing executed to first thing excuted

#### start
call self.connection.start() instead of super().start()

####  main
allow address and port to be specified by command line arguments
sleep(1) instead of sleep(2)

#### plan_path
new.  generates waypoints and send them to simulator with send_waypoints

This code set some constants (TARGET_ALTITUDE, and SAFETY_DISTANCE) used in the construction
of the obstacles vs free grid.
It next loads in the colliders.csv file in to 'data' skipping the first two rows.
The skipped rows being the home lat/log and the labels for the obstacle data.
'create_grid' is called to determine which grid cells are covered by obstructions (or
a safety margin around them) and which grid cells are available for flying the drone.
The start location is set to the current position.
The goal location is set to a go 10m North and 10m East.
Both locations are translated from a map centered on (0,0) to the collision grid
coordinate system with (0,0) in the South-West corner.
'a_star' is called to use the A* algoritm to search the free space for a path between
the start and the goal.
Only movement North, East, South, and West, all with a cost of 1, are allowed.
The heuristic function passed to the A* algorith is the vector norm 'linalg.norm', which in our
two dimensional space is equivalent to sqrt(x^2+y^2).
The return path path is translated from grid coordiate system to map centered coordinate
system waypoints by adding the offset between the origins of the two systems.
Finally the waypoints are sent to the simulator to draw out the path in green balls and
white lines.

##### send_waypoints

new.  called from plan_path. use msgpack.dumps to write waypoints as data to simulator


### Implementing Your Path Planning Algorithm [IYPPA]

#### [IYPPA-1] set_home_postion

This rubric item is marked in file motion_planning.py with comment IYPPA-1.

The first line of colliders.csv is 'lat0 37.792480, lon0 -122.397450'.
The file is opened in a with clause so it will be closed for the np.loadtxt.
readline() reads the line as a string. rstrip() removes the newline at the end.
str.replace() is used to remove the 'lat0' and 'lon0' characters.
The remaining character string is split on the ','.  float() is used to
convert the strings to a floating point numbers.

#### [IYPPA-2] determine your local position relative to global home

This rubric item is marked in file motion_planning.py with comment IYPPA-2.

global_position is set to [self._longitude, self._latitude, self._altitude]
global_to_local(global_position, self.global_home) determines the current
local position.

#### [IYPPA-3] change start point for planning to current local position

This rubric item is marked in file motion_planning.py with comment IYPPA-3.

north_offset and east_offset are the vector to (0,0) of the grid.  This
vector is subtracted from the the current local position in the map
reference frame to get the start position in the grid reference frame.

```
grid_start = (int(current_local_pos[0]-north_offset), int(current_local_pos[1]-east_offset))
```

#### [IYPPA-4 ] add flexibility to the desired goal location

This rubric item is marked in file motion_planning.py with comment IYPPA-4.

```
        goal_local_pos = global_to_local([-122.401902,37.794409,self.global_home[2]],self.global_home)
        grid_goal = (int(goal_local_pos[0]-north_offset), int(goal_local_pos[1]-east_offset))
```

The goal location can be changed by replacing `-122.401902,37.794409` with the desired longitude,
latitude.  global_to_local() translates the geotic coordinates to the local map reference frame.
grid_goal is set by translating the local map reference coordinates into grid coordiantes by
subtracting off the offset vector (north_offset, east_offset)

The original goal of lat 37.797330, lon -122.402224 (in the simulator world) of the original
rubric is an alley way between buildings of the United States Immigrations and Customs
Enforcement group in the real world.  While it is possible to fly into this space without
a physical collision, it would be nevertheless unwise to set as a goal without explicit permision
of this organization. Because of misalignment of the colliders and the building in the simulator,
a known problem, the SAFETY_DISTANCE = 5 when using the grid based Astar method instead of
SAFETY_DISTANCE = 3 when using the medial axis.

#### [IYPPA-5] Write your search algorithm

This rubric item is marked in files motion_planning.py, medial_axis_utils.py, and
planning_utils.py with comment IYPPA-5.

The diagonal motions on the grid method of A* are  achieved by adding the actions
NE, NW, SE, and SW with cost of sqrt(2).  valid_actions() is also modified to
remove NE, NW, SE, or SW when it would move off the grid, or collide.

The SAFETY margin was increased to 5m due to some alignemnt problems with the
collision data and the simulator.  This was necessary because the grid method
tends to plan close to the edge of the grid.

An alternative A* method using medial axis was implementented.  The code used
was essentially the code presented in lectures. Specifically Lesson 6, section
13. 'Medial Axis Exercise'.   This method plans considerably faster than A* grid
method.  Since it tends to stay as far away from collisions as possible, the
SAFETY limit was left at the original 3m. This code can be selected by changing
the METHOD = AStarMethod.GRID to METHOD = AStarMethod.MEDIAL_AXIS

However METHOD = AStarMethod.GRID was used since it produces staighter waypaths
which prune more pretty making it easier for reviewers to realize that the pruning
algorithm was cut and paste from the solution provided in class.

#### [IYPPA]-6] Cull waypoints from the path

This rubric item is marked in files motion_planning.py and
pruning_utils.py with comment IYPPA-6.

The prunning was done by using a collinearity test, essentially the code
presented in lectures. Specifically Lesson 6, sections 5 and 9.

### Executing the flight [Etf]

The way points have been successfully loaded onto the simulator and run.
A modification was made to the translation from path to waypoints to convert
values from numpy.int64 to native python int to allow for serialization.

[//]: # (Mark Anderson // Feb 2018 cohort // 2018_03Mar_10)
