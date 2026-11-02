"""Compatibility entry point for the Markdown-based report renderer."""
from pathlib import Path
import shutil
import subprocess
import sys


if __name__ == "__main__":
    node = shutil.which("node")
    if node is None:
        raise SystemExit("Node.js is required. See output/pdf/BUILD.md.")
    script = Path(__file__).with_suffix(".mjs")
    raise SystemExit(subprocess.call([node, str(script), *sys.argv[1:]]))
