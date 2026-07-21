#!/usr/bin/env python3
import sys
import os
import yaml
import re
from pathlib import Path

# Map optimizer parameter aliases to actual key names in YAML files
ALIAS_MAP = {
    "tire.longitudinal_peak_pdx1": "PDX1",
    "tire.longitudinal_stiffness_pkx1": "PKX1",
    "tire.longitudinal_curvature_pex1": "PEX1",
    "tire.lateral_peak_pdy1": "PDY1",
    "tire.lateral_stiffness_pky1": "PKY1",
    "tire.lateral_curvature_pey1": "PEY1",
    "tire.structural_longitudinal_stiffness": "LONGITUDINAL_STIFFNESS",
    "tire.structural_lateral_stiffness": "LATERAL_STIFFNESS",
    "tire.structural_yaw_stiffness": "YAW_STIFFNESS",
    "tire.longitudinal_peak_scale": "LMUX",
    "tire.longitudinal_stiffness_scale": "LKX",
    "tire.lateral_shape_scale": "LCY",
    "tire.lateral_peak_scale": "LMUY",
    "tire.lateral_curvature_scale": "LEY",
    "tire.lateral_stiffness_scale": "LKY",
    "tire.combined_slip_lateral_stiffness_scale": "LYKA",
}

def update_file_parameter(config_file: Path, target_key: str, new_value) -> bool:
    if not config_file.exists():
        return False

    with open(config_file, 'r') as f:
        lines = f.readlines()

    # Regex to find:   target_key: <number> [optional comment]
    pattern = re.compile(r'^(\s*' + re.escape(target_key) + r'\s*:\s*)([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)(.*)$')

    replaced = False
    for i, line in enumerate(lines):
        match = pattern.match(line)
        if match:
            prefix = match.group(1)
            suffix = match.group(3)

            if isinstance(new_value, float):
                val_str = f"{new_value:.6g}"
            else:
                val_str = str(new_value)

            lines[i] = f"{prefix}{val_str}{suffix}\n"
            replaced = True
            break

    if replaced:
        with open(config_file, 'w') as f:
            f.writelines(lines)

    return replaced

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
    
    updated_files = set()

    for full_key, new_value in params.items():
        parts = full_key.split('.')
        if len(parts) != 2:
            print(f"Warning: Skipping malformed parameter key '{full_key}'. Expected format 'category.parameter'")
            continue

        category, raw_param_name = parts
        target_key = ALIAS_MAP.get(full_key, raw_param_name)

        if category == "car":
            config_file = car_config_dir / f"{car_name}.yaml"
        else:
            model_dir = car_config_dir / f"{category}_model"
            config_file = model_dir / f"{car_name}_{category}.yaml"

        if update_file_parameter(config_file, target_key, new_value):
            updated_files.add(config_file)
            val_str = f"{new_value:.6g}" if isinstance(new_value, float) else str(new_value)
            print(f"Updated {full_key} (as {target_key}) -> {val_str} in {config_file.name}")
        else:
            print(f"Warning: Parameter '{target_key}' ({full_key}) not found in {config_file}. Skipping.")

        # Also update main car yaml if gear_ratio is changed
        if category == "transmission" and target_key == "gear_ratio":
            main_car_file = car_config_dir / f"{car_name}.yaml"
            if update_file_parameter(main_car_file, target_key, new_value):
                updated_files.add(main_car_file)
                val_str = f"{new_value:.6g}" if isinstance(new_value, float) else str(new_value)
                print(f"Updated main car config {full_key} -> {val_str} in {main_car_file.name}")

    print(f"\nSuccessfully applied parameters to {len(updated_files)} configuration files!")

if __name__ == "__main__":
    main()

