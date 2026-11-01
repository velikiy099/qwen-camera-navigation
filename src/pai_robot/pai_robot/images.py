"""ROS RGB/BGR image bytes directly to PNG. No ROS or imaging-library imports."""
from __future__ import annotations
import struct
import zlib

CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}

def png_from_image(data: bytes, width: int, height: int, step: int, encoding: str) -> bytes:
    if encoding not in CHANNELS:
        raise ValueError(f'Unsupported image encoding {encoding!r}; expected rgb8/bgr8/rgba8/bgra8')
    if width <= 0 or height <= 0 or width * height > 4096 * 4096:
        raise ValueError('Invalid or excessive image geometry')
    channels = CHANNELS[encoding]
    if step < width * channels or len(data) < step * height:
        raise ValueError('Image payload or row stride is too short')
    def chunk(tag: bytes, payload: bytes) -> bytes:
        return struct.pack('!I', len(payload)) + tag + payload + struct.pack('!I', zlib.crc32(tag + payload) & 0xffffffff)
    rgb = bytearray()
    for y in range(height):
        rgb.append(0)  # PNG filter: none
        row = data[y * step:y * step + width * channels]
        if encoding == 'rgb8':
            rgb.extend(row)
        else:
            for x in range(width):
                pixel = row[x * channels:x * channels + 3]
                rgb.extend(pixel if encoding == 'rgba8' else pixel[::-1])
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(bytes(rgb), 3)) + chunk(b'IEND', b''))
