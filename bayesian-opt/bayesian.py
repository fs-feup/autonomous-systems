import subprocess
import time
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from skopt import Optimizer

# Define the parameter space as a list of `Real` objects, specifying the range for each parameter.
param_space = [
    Real(0.001, 0.1, name="param1"),  # Example: Learning rate or some parameter
    Real(
        100, 1000, name="param2"
    ),  # Example: Number of iterations or some other parameter
    Real(0.1, 10, name="param3"),  # Example: Another hyperparameter
]


# Define a wrapper function to call the C++ program and pass the parameters.
def call_cpp_program(params):
    # Convert the parameters to a string format to send them to the C++ executable.
    param_str = " ".join(map(str, params))

    try:
        # Use subprocess to call the C++ executable and pass the parameters.
        result = subprocess.run(
            [
                "./my_cpp_executable",
                param_str,
            ],  # Replace with your actual executable path
            capture_output=True,
            text=True,
            check=True,
        )

        # The result should be the output from your C++ program (a numeric value).
        return float(result.stdout.strip())
    except subprocess.CalledProcessError as e:
        # Handle errors from the C++ execution if any.
        print("Error while running C++ program:", e)
        return float("inf")  # Return a large value to signify failure


# Define the objective function to minimize, which will call the C++ program.
def objective_function(params):
    # Call the C++ program with the parameter values and return the result (the value to minimize).
    result = call_cpp_program(params)
    return result


# Initialize the Bayesian optimizer with the parameter space.
optimizer = Optimizer(param_space, "GP", n_initial_points=10, random_state=None)

# Initial parameters for tracking best result
best_params = None
best_value = float("inf")

# Run indefinitely
while True:
    # Ask the optimizer for the next set of parameters to evaluate.
    next_params = optimizer.ask()

    # Evaluate the objective function (call the C++ program with the proposed parameters).
    current_value = objective_function(next_params)

    # Tell the optimizer about the result so it can update its model.
    optimizer.tell(next_params, current_value)

    # Update the best result found so far if this evaluation is better.
    if current_value < best_value:
        best_value = current_value
        best_params = next_params

    # Print the current best parameters and result.
    print(f"Best parameters so far: {best_params}")
    print(f"Best objective value (to minimize): {best_value}")

    # Sleep for a short time before the next iteration (optional)
    time.sleep(1)
