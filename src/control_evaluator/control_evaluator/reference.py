import math


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _distance_and_interpolation_to_segment(
    current_x: float, current_y: float, start: tuple, end: tuple
) -> tuple[float, float]:
    start_x, start_y, _, _ = start
    end_x, end_y, _, _ = end

    segment_x = end_x - start_x
    segment_y = end_y - start_y
    segment_len_sq = segment_x * segment_x + segment_y * segment_y

    if segment_len_sq <= 1e-12:
        distance = math.sqrt(
            (current_x - start_x) ** 2 + (current_y - start_y) ** 2
        )
        return distance, 0.0

    interpolation = (
        (current_x - start_x) * segment_x + (current_y - start_y) * segment_y
    ) / segment_len_sq
    interpolation = max(0.0, min(1.0, interpolation))

    proj_x = start_x + interpolation * segment_x
    proj_y = start_y + interpolation * segment_y
    distance = math.sqrt((current_x - proj_x) ** 2 + (current_y - proj_y) ** 2)

    return distance, interpolation


def compute_path_reference(
    current_x: float, current_y: float, path_points: list
) -> tuple[float, float, float]:
    """Return (distance_error, ref_heading, ref_velocity) for the given position.

    Path points are (x, y, v, orientation) tuples. Orientation is interpolated
    along the closest segment using the stored yaw values directly.
    """
    if len(path_points) == 1:
        px, py, pv, po = path_points[0]
        distance = math.sqrt((current_x - px) ** 2 + (current_y - py) ** 2)
        return distance, po, pv

    best_index = 0
    best_dist_sq = float("inf")
    for index, (px, py, _, _) in enumerate(path_points):
        dx = current_x - px
        dy = current_y - py
        dist_sq = dx * dx + dy * dy
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_index = index

    candidates = []
    if best_index > 0:
        candidates.append((path_points[best_index - 1], path_points[best_index]))
    if best_index < len(path_points) - 1:
        candidates.append((path_points[best_index], path_points[best_index + 1]))

    best = None
    for start, end in candidates:
        distance, interpolation = _distance_and_interpolation_to_segment(
            current_x, current_y, start, end
        )
        if best is None or distance < best[0]:
            best = (distance, interpolation, start, end)

    if best is None:
        px, py, pv, po = path_points[best_index]
        distance = math.sqrt((current_x - px) ** 2 + (current_y - py) ** 2)
        return distance, po, pv

    distance, interpolation, start, end = best
    _, _, start_v, start_orientation = start
    _, _, end_v, end_orientation = end

    ref_heading = normalize_angle(
        start_orientation + interpolation * normalize_angle(end_orientation - start_orientation)
    )
    ref_velocity = start_v + interpolation * (end_v - start_v)
    return distance, ref_heading, ref_velocity
