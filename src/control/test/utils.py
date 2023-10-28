import os 
import csv 

EXEC_TIME_OUTPUT_FILE = os.path.dirname(os.path.abspath(__file__)) + \
    '/../../performance/exec_time/control.csv'


#! 
# @brief Saves execution time results in csv file
def save_exec_time(module : str, scope : str, scenario : str, exec_time : float):
    mode = 'a' if os.path.exists(EXEC_TIME_OUTPUT_FILE) else 'w'
    with open(EXEC_TIME_OUTPUT_FILE, mode) as f:
        writer = csv.writer(f)
        if mode == 'w':
            writer.writerow(['Module', 'Scope', 'Scenario', 'Execution Time (ms)'])
        writer.writerow([module, scope,\
            scenario, exec_time])