from PID import PID
from control import simulate, evaluate_temperature_control
import os
import csv

def evaluate_pid_grid_and_save_csv(
    kp_range, ki_range, kd_range, 
    setpoint=50, window_percent=1, 
    csv_path="out/pid_eval.csv"
):

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Kp', 'Ki', 'Kd', 'Score', 'InWindow', 'Total'])

        for kp in kp_range:
            for ki in ki_range:
                for kd in kd_range:
                    pid = PID(kp, ki, kd)
                    times, temps, power = simulate(pid)
                    score, in_window, total = evaluate_temperature_control(
                        temps, setpoint=setpoint, window_percent=window_percent
                    )
                    writer.writerow([kp, ki, kd, score, in_window, total])

    print(f"Results saved to {csv_path}")

# Example usage:
if __name__ == "__main__":
    import numpy as np
    # Adapted ranges with smaller steps
    kp_range = np.arange(2.5, 3.01, 0.1)  
    ki_range = np.arange(0.0, 1.01, 0.1)  
    kd_range = np.arange(0.0, 0.21, 0.05)  
    evaluate_pid_grid_and_save_csv(kp_range, ki_range, kd_range)