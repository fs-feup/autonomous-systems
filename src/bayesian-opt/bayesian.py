import sys
import os
import signal
import subprocess
import importlib
from skopt import Optimizer
from operator import itemgetter

algorithm_to_tune = "cone_coloring"

# Base directory where bayesian.py is located
base_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the full path to the subdirectory
full_path = os.path.join(base_dir, algorithm_to_tune)

# Check if the adapt.py exists in the specified directory
if os.path.exists(os.path.join(full_path, "adapt.py")):
    # Append the subdirectory to sys.path
    sys.path.append(full_path)

    # Dynamically import the "adapt" module (without the .py extension)
    module = importlib.import_module("adapt")
else:
    print(f"adapt.py not found in {algorithm_to_tune}")


executable_path = "./" + algorithm_to_tune + "/bayesian_opt"
export_path = algorithm_to_tune + "/parameters_rank.txt"


# Define a wrapper function to call the C++ program and pass the parameters.
def call_cpp_program(params):
    # Convert the parameters to a string format to send them to the C++ executable.
    param_str_list = list(map(str, params))

    try:
        # Use subprocess to call the C++ executable and pass the parameters.
        result = subprocess.run(
            [
                executable_path,
                *param_str_list,
            ],
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


# To track the top 5 sets of parameters and their values.
top_5 = []

best_value = float("inf")
best_params = module.parameters_list[0]


def update_top_5(params, value):
    # Append the new parameters and value to the list.
    top_5.append((params, value))

    # Sort the list based on the objective value (second item in the tuple).
    top_5.sort(key=itemgetter(1))

    # Keep only the top 5 values.
    if len(top_5) > 5:
        top_5.pop()


# Function to export the top 5 parameters to a file.
def export_top_5_to_file():
    with open(export_path, "a") as f:
        f.write("Top 5 parameter sets and their objective values:\n")
        for i, (params, value) in enumerate(top_5, 1):
            f.write(f"Rank {i}: Parameters = {params}, Objective Value = {value}\n")
    print("\nTop 5 parameters have been saved to 'parameters_rank.txt'.")


# Signal handler to catch KeyboardInterrupt and perform cleanup.
def signal_handler(sig, frame):
    print("\nTermination signal received. Exporting top 5 parameters...")
    export_top_5_to_file()
    sys.exit(0)


# Register the signal handler for KeyboardInterrupt (Ctrl+C).
signal.signal(signal.SIGINT, signal_handler)


# Initialize the Bayesian optimizer with the parameter space.
optimizer = Optimizer(module.param_space, "GP", n_initial_points=10, random_state=None)

for params in module.parameters_list:
    result = objective_function(params)
    optimizer.tell(params, result)
    update_top_5(params, result)
    if result < best_value:
        best_value = result
        best_params = params
        # Print the current best parameters and result.
        print(f"New best parameters: {best_params}")
        print(f"New best objective value (to minimize): {best_value}")

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
        print(f"New best parameters: {best_params}")
        print(f"New best objective value (to minimize): {best_value}")

    update_top_5(next_params, current_value)
