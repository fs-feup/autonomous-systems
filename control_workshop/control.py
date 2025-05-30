import matplotlib.pyplot as plt
import os
from PID import PID 
# PID Controller

# Simulated System (e.g. a room with a heater)
def simulate(pid, initial_temp=20, setpoint=50, dt=0.1, total_time=60, ambient_temp=15, k=0.1):
    temp = initial_temp
    temps = []
    times = []
    power_output = []

    for t in range(int(total_time / dt)):
        time = t * dt
        
        control = pid.update(setpoint, temp, dt)
        heating = 0.5 * control
        
        cooling = k * (temp - ambient_temp)
        temp += (heating - cooling) * dt
        
        temps.append(temp)
        power_output.append(heating)
        times.append(time)

    return times, temps, power_output

def evaluate_temperature_control(temps, setpoint=50, window_percent=1):
    window = setpoint * (window_percent / 100)
    lower_bound = setpoint - window
    upper_bound = setpoint + window

    within_window = [lower_bound <= temp <= upper_bound for temp in temps]
    score = sum(within_window) / len(temps) if temps else 0.0

    return score, sum(within_window), len(temps)

if __name__ == "__main__":
    # Set up PID controller parameters
    Kp = 2.9  # Proportional gain
    Ki = 0.4  # Integral gain
    Kd = 0.0  # Derivative gain

    # Initialize PID controller
    pid = PID(Kp, Ki, Kd)

    # Run simulation
    times, temps, power = simulate(pid)
    score, in_window, total = evaluate_temperature_control(temps, setpoint=50, window_percent=1)
    print(f"Pontuação: {score:.2f} ({in_window}/{total} steps within ±1% of setpoint)")

    os.makedirs("out", exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(times, temps, label='Temperature (°C)')
    plt.axhline(y=50, color='r', linestyle='--', label='Setpoint')
    plt.legend()
    plt.title('Temperature vs Time')

    plt.subplot(2, 1, 2)
    plt.plot(times, power, label='Heater Power')
    plt.legend()
    plt.title('Control Effort')

    plt.tight_layout()
    plt.savefig("out/plot.png")
