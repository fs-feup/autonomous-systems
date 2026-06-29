from pathlib import Path
from sys import prefix
import yaml
from typing import Optional

class Params:
    def __init__(self):
        self.adapter = "vehicle"  # Default adapter
        self.using_simulated_slam = False
        self.using_simulated_se = False
        self.using_simulated_lap_count = False
        self._load_params_from_global_config()

    def _load_params_from_global_config(self) -> str:
        resolved_path = self._resolve_global_config_path()

        if resolved_path is None:
            raise FileNotFoundError(
                "Global config file not found. Please provide a valid global_config_path parameter."
            )

        with resolved_path.open("r", encoding="utf-8") as handle:
            cfg = yaml.safe_load(handle) or {}
    
        adapter = str(cfg.get("global", {}).get("adapter", "vehicle")).strip()
        using_simulated_slam = bool(cfg.get("global", {}).get("use_simulated_se", False))
        using_simulated_se = bool(cfg.get("global", {}).get("use_simulated_velocities", False))
        using_simulated_lap_count = bool(cfg.get("global", {}).get("use_simulated_se", False))
        if adapter:
            self.adapter = adapter
        if using_simulated_slam:
            self.using_simulated_slam = using_simulated_slam
        if using_simulated_se:
            self.using_simulated_se = using_simulated_se
        if using_simulated_lap_count:
            self.using_simulated_lap_count = using_simulated_lap_count

    def _resolve_global_config_path(self) -> Optional[Path]:
        relative_target = Path("config") / "global" / "global_config.yaml"
        search_roots = [Path.cwd(), Path(__file__).resolve()]
        checked = set()

        for root in search_roots:
            for parent in [root, *root.parents]:
                candidate = (parent / relative_target).resolve()
                if candidate in checked:
                    continue
                checked.add(candidate)
                if candidate.exists():
                    return candidate

        fallback = Path("/home/ws/config/global/global_config.yaml")
        if fallback.exists():
            return fallback

        return None