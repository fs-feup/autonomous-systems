# costmap for the MPPI algorithm in the form of a 2D image

# the bounds give the scale and limits of the image
float64 x_bounds_min
float64 x_bounds_max
float64 y_bounds_min
float64 y_bounds_max

# he pixels per meter are used in scaling the image up
float64 pixels_per_meter

# the costmap contains 4 channels of floats
float32[] channel0
float32[] channel1
float32[] channel2
float32[] channel3

