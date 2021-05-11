# The response should give the parameters that can be used for any
# parametric gait see https://confluence.projectmarch.nl:8443/display/62tech/Obstacles+analysis
# for more explanation about the range of heights and step sizes

# Parameter that is used for the first interpolation, between 0 and 1
#
# Stairs: Determines step size, min depth is 20 cm, max is ? cm
# Ramp: Determines gradient, min gradient is 5 degrees, max is  20 degrees
float64 first_parameter


# Parameter that is used for the second interpolation, if obstacle is done with 2D
# interpolation.
#
# Stairs: Determines the step height, min height is 12 cm, max height is 22 cm
# Ramp: Not used for the ramp gait, since this is currently in 1D
float64 second_parameter

# Parameter that influences the amount the step should go sideways (using HAA mostly)
# for the step chosen by the realsense, between 0 and 1, where 0 is regular angle and 1
# is maximal sideways step for this obstacle. Currently, we have not implemented
# this in the gaits yet.
#
# Stairs: Not used yet, should always be 0
# Ramp: Not used yet, should always be 0
float64 side_step_parameter