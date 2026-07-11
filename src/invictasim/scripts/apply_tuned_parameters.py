#!/usr/bin/env python3
import sys
import os
import yaml
import re
from pathlib import Path

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 apply_tuned_parameters.py <best_parameters.yaml> <car_config_dir> [car_name]")
        print("Example: python3 apply_tuned_parameters.py src/invictasim/tuning_csvs/best_parameters.yaml config/car 02")
        sys.exit(1)

    best_params_path = Path(sys.argv[1])
    car_config_dir = Path(sys.argv[2])
    car_name = sys.argv[3] if len(sys.argv) > 3 else "02"

    if not best_params_path.exists():
        print(f"Error: {best_params_path} not found.")
        sys.exit(1)

    with open(best_params_path, 'r') as f:
        best_data = yaml.safe_load(f)

    if 'parameters' not in best_data:
        print("Error: 'parameters' key not found in the tuned parameters file.")
        sys.exit(1)

    params = best_data['parameters']
    
    # Map to track which files were updated
    updated_files = set()

    for full_key, new_value in params.items():
        parts = full_key.split('.')
        if len(parts) != 2:
            print(f"Warning: Skipping malformed parameter key '{full_key}'. Expected format 'category.parameter'")
            continue
            
        category, param_name = parts
        
        # Determine the file path
        # Example: motor -> config/car/motor_model/02_motor.yaml
        model_dir = car_config_dir / f"{category}_model"
        config_file = model_dir / f"{car_name}_{category}.yaml"
        
        if not config_file.exists():
            print(f"Warning: Config file {config_file} not found for parameter {full_key}. Skipping.")
            continue
            
        # Read file lines to perform regex replacement (to preserve comments and formatting)
        with open(config_file, 'r') as f:
            lines = f.readlines()
            
        # Regex to find:   param_name: <number> [optional comment]
        # We capture the prefix (indentation + param_name + colon + space) and the suffix (comments etc)
        # Note: This handles scientific notation and floats
        pattern = re.compile(r'^(\s*' + re.escape(param_name) + r'\s*:\s*)([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)(.*)$')
        
        replaced = False
        for i, line in enumerate(lines):
            match = pattern.match(line)
            if match:
                prefix = match.group(1)
                suffix = match.group(3)
                
                # Format the new value cleanly
                if isinstance(new_value, float):
                    val_str = f"{new_value:.6g}" # Avoid excessive decimal places
                else:
                    val_str = str(new_value)
                    
                lines[i] = f"{prefix}{val_str}{suffix}\n"
                replaced = True
                updated_files.add(config_file)
                print(f"Updated {full_key} -> {val_str} in {config_file.name}")
                break
                
        if not replaced:
            print(f"Warning: Parameter '{param_name}' not found in {config_file}. Skipping.")
            
        if replaced:
            with open(config_file, 'w') as f:
                f.writelines(lines)

    print(f"\nSuccessfully applied parameters to {len(updated_files)} configuration files!")

if __name__ == "__main__":
    main()
