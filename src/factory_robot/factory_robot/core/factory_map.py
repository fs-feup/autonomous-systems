from __future__ import annotations
from typing import List, Tuple, Optional, Dict

# Map element constants
FLOOR = '0'
WALL = '9'
SHELVES = {'A', 'B', 'C', 'D'}
BOXES = {'1', '2', '3', '4'}
START = 'S'


class FactoryMap:
    """
    Loads and holds the factory grid. Coordinates are (row, col).
    Provides helpers for collision checks and item interactions.
    """

    def __init__(self, path: str):
        self.path = path
        self.grid: List[List[str]] = []
        self.start: Optional[Tuple[int, int]] = None

        # Quick lookup sets
        self.box_positions: Dict[Tuple[int, int], str] = {}
        self.shelf_positions: Dict[Tuple[int, int], str] = {}

        self._load()

    def _load(self) -> None:
        self.grid.clear()
        self.box_positions.clear()
        self.shelf_positions.clear()
        self.start = None

        with open(self.path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                self.grid.append(list(line))

        for r, row in enumerate(self.grid):
            for c, ch in enumerate(row):
                if ch == START:
                    self.start = (r, c)
                elif ch in BOXES:
                    self.box_positions[(r, c)] = ch
                elif ch in SHELVES:
                    self.shelf_positions[(r, c)] = ch

        if self.start is None:
            raise ValueError("Map must contain a start 'S' position.")

    # --- helpers ---
    @property
    def rows(self) -> int:
        return len(self.grid)

    @property
    def cols(self) -> int:
        return len(self.grid[0]) if self.grid else 0

    def in_bounds(self, rc: Tuple[int, int]) -> bool:
        r, c = rc
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_walkable(self, rc: Tuple[int, int]) -> bool:
        """Walkable if in-bounds and not a wall/shelf/box."""
        if not self.in_bounds(rc):
            return False
        ch = self.grid[rc[0]][rc[1]]
        if ch == WALL:
            return False
        if ch in SHELVES:
            return False
        if ch in BOXES:
            return False
        # FLOOR or START are walkable
        return True

    def neighbors_4(self, rc: Tuple[int, int]) -> List[Tuple[int, int]]:
        r, c = rc
        cand = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
        return [p for p in cand if self.in_bounds(p)]

    def adjacent_box(self, rc: Tuple[int, int]) -> Optional[Tuple[Tuple[int, int], str]]:
        """Return ((r,c), box_char) if a box is in 4-neighborhood."""
        for nb in self.neighbors_4(rc):
            if nb in self.box_positions:
                return nb, self.box_positions[nb]
        return None

    def adjacent_shelf(self, rc: Tuple[int, int]) -> Optional[Tuple[Tuple[int, int], str]]:
        """Return ((r,c), shelf_char) if a shelf is in 4-neighborhood."""
        for nb in self.neighbors_4(rc):
            if nb in self.shelf_positions:
                return nb, self.shelf_positions[nb]
        return None

    # --- mutations for boxes ---
    def remove_box(self, rc: Tuple[int, int]) -> Optional[str]:
        """Remove a box at rc, return its char or None."""
        if rc in self.box_positions:
            ch = self.box_positions.pop(rc)
            self.grid[rc[0]][rc[1]] = FLOOR
            return ch
        return None

    def add_box(self, rc: Tuple[int, int], ch: str) -> bool:
        """
        Place a box char ('1'..'4') on a FLOOR tile. Returns True if placed.
        We never put boxes on shelves; 'drop' near shelf just consumes the held box.
        """
        if ch not in BOXES:
            return False
        if not self.in_bounds(rc):
            return False
        if self.grid[rc[0]][rc[1]] != FLOOR:
            return False
        self.box_positions[rc] = ch
        self.grid[rc[0]][rc[1]] = ch
        return True

    # --- rendering ---
    def render_without_robot(self) -> str:
        """
        Return the map as multi-line string, with 'S' shown as FLOOR.
        (Robot is not painted here; the node can overlay 'R' if needed elsewhere.)
        """
        out: List[str] = []
        for r, row in enumerate(self.grid):
            line = []
            for c, ch in enumerate(row):
                line.append(FLOOR if ch == START else ch)
            out.append(''.join(line))
        return '\n'.join(out)
