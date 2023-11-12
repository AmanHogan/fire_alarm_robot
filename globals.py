"""Contains constants and varaibles that will be modified before compile time"""

################################ COMPILE TIME CONSTANTS ####################

# error factor when moving on surface
TILE_ERROR_DISTANCE = 1.00
ERROR_FACTOR_DISTANCE = TILE_ERROR_DISTANCE # Change this as needed

# error factor when moving on surface
TILE_ERROR_TURN = .950
ERROR_FACTOR_TURN = TILE_ERROR_TURN # Change this as needed
#########################################################################

####################### ROBOT CONSTANTS #################################

TIRE_CIRCUMFERENCE = 178 # Circumference of the tire in [mm]

FULL_ROTATION = 360 # [#]
TIRE_RPM = 250 # Revolutions per minute [r/min] # Change this as needed
ROBOT_LENGTH = 105 # Length of the robot in [mm]
DIST_BTWN_WHEELS = 158.0000 # Distance between the wheels in [mm]
ROBOT_RADIUS_MM = (DIST_BTWN_WHEELS/2.0000) # Radius of robot tire axel [mm]
M_PI = 3.14159265359  # pi constant [#]

###########################################################################
MIN_WALL_DISTANCE = 200
FIRE_LIGHT_INTENSITY = 35
#########################################################################