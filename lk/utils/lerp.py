import numpy as np


def lerp_list(start: list[float], end: list[float], fraction: float) -> list[float]:

    if start is None or (isinstance(start, list) and len(start) == 0):
        return end

    if end is None or (isinstance(end, list) and len(end) == 0):
        return start

    start_arr = np.array(start)
    end_arr = np.array(end)

    blend_arr = (
        start_arr + (end_arr - start_arr) * fraction
    )  # blend from start_val -> end_val
    return blend_arr.tolist()


def lerp_value(start: float, end: float, fraction: float) -> float:
    return start + (end - start) * fraction
