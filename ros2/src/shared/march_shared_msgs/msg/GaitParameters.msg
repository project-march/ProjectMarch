# The response should give the parameters that can be used for any
# parametric gait see https://confluence.projectmarch.nl:8443/display/62tech/Obstacles+analysis
# for more explanation about the range of heights and step sizes

# Parameter that determines the step size,
# between 0 and 1, where 0 is the small step for this obstacle and
# 1 is the large step gait for this obstacle.
#
# Stairs: Min depth is 20 cm, max is ? cm
# Ramp: Will not be used for ramp up/down
float64 step_size_parameter


# Parameter that determines the height of the step to make,
# also between 0 and 1, where 0 is the low step for this obstacle and
# 1 is the high step gait for this obstacle.
#
# Stairs: Min height is 12 cm, max height is 22 cm
# Ramp: Min gradient is 5 degrees, max is  20 degrees
float64 step_height_parameter

# Parameter that influences the amount the step should go sideways (using HAA mostly)
# for the step chosen by the realsense, between 0 and 1, where 0 is regular angle and 1
# is maximal sideways step for this obstacle. Currently, we have not implemented
# this in the gaits yet.
#
# Stairs: Not used yet, should always be 0
# Ramp: Not used yet, should always be 0
float64 side_step_parameter