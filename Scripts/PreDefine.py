#! /usr/bin/env python3

PI = 3.14159265
THRESHOLD_SWITCH = 0.0025
dt = 0.05
MAX_ANGLE_DEVIATION = 8
GRID_SIZE = 9
GOAL_X = 5
GOAL_Y = 4
WALL_DETECT_DIST = 1.0
OPEN_DETECT_DIST = 1.5
MIN_X = 0.0
MIN_Y = 0.0
RESOLUTION = 1.0

#FIXED PARAM
UNDEFINED = -1
NORTH = 0
EAST = 1
SOUTH = 2
WEST =3
OPEN = 0
WALL = 1
GOAL_NOT_REACH = 0
GOAL_REACH = 1
GoalCoord = (GOAL_X, GOAL_Y)
StartCoord = (0, 0)


Kp_a = 0.5
Ki_a = 0.2
Kd_a = 0.2
Kp_x = 0.7
Ki_x = 0.3
Kd_x = 0.1
dt = 0.1
