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
TODO: expain in broad strokes what plan path does

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
convert the string to a floating point number

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

#### [IYPPA-5] Write your search algorithm

#### [IYPPA]-6] Cull waypoints from the path

### Executing the flight [Etf]

[//]: # (Mark Anderson // Feb 2018 cohort // 2018_03Mar_10)
