from skopt.space import Real

# Define the parameter space as a list of `Real` objects, specifying the range for each parameter.
param_space = [
    Real(0, 20, name="angle_weight"),
    Real(0, 20, name="distance_weight"),
    Real(0, 20, name="ncones_weight"),
    Real(0, 10, name="distance_exponent"),
    Real(0, 10, name="angle_exponent"),
    Real(0, 80, name="max_cost"),
]

params1 = [
    8.089748165412072,
    4.481112717588674,
    6.538940942092987,
    0.7929921809618147,
    2.9846881916431918,
    64.55265403601587,
]

params2 = [
    0.0,
    14.16236432745398,
    5.33282693020819,
    0.8069423579816568,
    4.31592836948307,
    80.0,
]
params3 = [
    7.999146341232352,
    10.867903810813948,
    7.59670853621077,
    0.7951361065701903,
    5.001991175710679,
    80.0,
]
params4 = [
    2.5036366399844723,
    9.230972680602905,
    6.93447117874158,
    0.7985945531629085,
    3.7688073553028274,
    80.0,
]
params5 = [
    3.033271334001328,
    8.561458806002841,
    6.713705336752474,
    0.7968794824887211,
    3.531145094725373,
    80.0,
]
params6 = [11, 8, 8.7, 0.698, 5.3, 40]
params7 = [
    1.913196876350798,
    16.26522041019923,
    8.818786857572633,
    0.697207576173325,
    6.196503788316914,
    74.6033253491355,
]
# Some specific parameters you may want to test
parameters_list = [params1, params2, params3, params4, params5, params6, params7]
