"""Fast pixel-level tracker for brightly colored boxes seen by robot camera.

This module is *traditional computer vision*, not ML. The learned vision-language
model in Ollama verifies targets from actual camera images before motion starts.
The tracker then supplies continuous pixel feedback between slow AI calls.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class ColorBlob:
    center_x_norm: float     # -1 means left edge; +1 means right edge
    pixel_fraction: float    # rough foreground fraction of full image
    width_fraction: float
    height_fraction: float
    bbox_xyxy: tuple[int, int, int, int]


def _is_target_pixel(b: int, g: int, r: int, target: str) -> bool:
    if target == "red":
        return r > 88 and r > 1.55 * g and r > 1.55 * b and r - g > 34
    if target == "blue":
        return b > 75 and b > 1.40 * g and b > 1.55 * r and b - r > 30
    raise ValueError("Only red or blue are supported")


def find_target_blob(bgra: bytes, width: int, height: int, target: str, stride: int = 3) -> ColorBlob | None:
    """Largest 8-connected blob of camera pixels matching a color.

    The ROS image adapter converts supported encodings into packed BGRA. This is NOT simulator
    object ground truth and uses neither the Supervisor API nor GPS.
    """
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ValueError("Invalid image geometry")
    if len(bgra) != width * height * 4:
        raise ValueError("Packed BGRA image must have width*height*4 bytes")
    if stride < 1:
        raise ValueError("stride must be >= 1")
    if target not in ("red", "blue"):
        raise ValueError("Expected red or blue target")

    columns = list(range(0, width, stride))
    rows = list(range(0, height, stride))
    grid_width = len(columns)
    grid_height = len(rows)
    mask = bytearray(grid_width * grid_height)
    for iy, y in enumerate(rows):
        for ix, x in enumerate(columns):
            index = 4 * (y * width + x)
            if _is_target_pixel(bgra[index], bgra[index + 1], bgra[index + 2], target):
                mask[iy * grid_width + ix] = 1

    selected = None
    for cell in range(len(mask)):
        if mask[cell] != 1:
            continue
        mask[cell] = 2
        queue = deque([cell])
        count = 0
        x_sum = 0
        min_x = width
        max_x = 0
        min_y = height
        max_y = 0
        while queue:
            position = queue.popleft()
            iy, ix = divmod(position, grid_width)
            x, y = columns[ix], rows[iy]
            count += 1
            x_sum += x
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            for next_y in range(max(0, iy - 1), min(grid_height, iy + 2)):
                for next_x in range(max(0, ix - 1), min(grid_width, ix + 2)):
                    next_cell = next_y * grid_width + next_x
                    if mask[next_cell] == 1:
                        mask[next_cell] = 2
                        queue.append(next_cell)
        if selected is None or count > selected[0]:
            selected = (count, x_sum, min_x, min_y, max_x, max_y)
    # At least 0.18% of the image, with enough samples for a true connected blob.
    min_count = max(8, int(0.0018 * width * height / (stride * stride)))
    if selected is None or selected[0] < min_count:
        return None
    count, x_sum, x0, y0, x1, y1 = selected
    if x1 - x0 < 8 or y1 - y0 < 6:
        return None
    return ColorBlob(
        center_x_norm=max(-1.0, min(1.0, 2 * (x_sum / count) / max(width - 1, 1) - 1)),
        pixel_fraction=min(1.0, count * stride * stride / (width * height)),
        width_fraction=(x1 - x0 + stride) / width,
        height_fraction=(y1 - y0 + stride) / height,
        bbox_xyxy=(x0, y0, min(width - 1, x1 + stride), min(height - 1, y1 + stride)),
    )
