"""ROS Image-compatible bytes -> packed BGRA -> PNG. No ROS imports."""
from __future__ import annotations
import struct
import zlib

CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}

def to_bgra(data: bytes, width: int, height: int, step: int, encoding: str) -> bytes:
    if encoding not in CHANNELS:
        raise ValueError(f'Unsupported image encoding {encoding!r}; expected rgb8/bgr8/rgba8/bgra8')
    if width <= 0 or height <= 0 or width * height > 4096 * 4096:
        raise ValueError('Invalid or excessive image geometry')
    channels = CHANNELS[encoding]
    if step < width * channels or len(data) < step * height:
        raise ValueError('Image payload or row stride is too short')
    output = bytearray(width * height * 4)
    for y in range(height):
        row = data[y * step:y * step + width * channels]
        dest = y * width * 4
        for x in range(width):
            i, j = x * channels, dest + x * 4
            a, b, c = row[i:i + 3]
            if encoding.startswith('rgb'):
                r, g, blue = a, b, c
            else:
                blue, g, r = a, b, c
            output[j:j + 4] = bytes((blue, g, r, row[i + 3] if channels == 4 else 255))
    return bytes(output)

def png_from_bgra(bgra: bytes, width: int, height: int) -> bytes:
    if width <= 0 or height <= 0 or len(bgra) != width * height * 4:
        raise ValueError('Invalid BGRA geometry')
    def chunk(tag: bytes, payload: bytes) -> bytes:
        return struct.pack('!I', len(payload)) + tag + payload + struct.pack('!I', zlib.crc32(tag + payload) & 0xffffffff)
    rgb = bytearray()
    for y in range(height):
        rgb.append(0)  # PNG filter: none
        for x in range(width):
            i = (y * width + x) * 4
            rgb.extend((bgra[i + 2], bgra[i + 1], bgra[i]))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(bytes(rgb), 3)) + chunk(b'IEND', b''))
