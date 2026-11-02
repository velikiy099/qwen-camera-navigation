"""Draw report-only, dimensioned top views directly from the two current SDF files."""
from pathlib import Path
import hashlib
import itertools
import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET

from reportlab.graphics import renderSVG
from reportlab.graphics.shapes import Drawing, Circle, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
FONT = 'HeiseiMin-W3'
INK = colors.HexColor('#172B3A')
MUTED = colors.HexColor('#52636F')
LABELS = {
    'red_box': ('赤い箱', (1.0, -1.2)),
    'red_sphere': ('赤い球', (2.0, -0.16)),
    'blue_box': ('青い箱', (1.0, 1.35)),
    'green_cylinder': ('緑の円柱', (2.05, 2.47)),
    'yellow_sphere': ('黄色い球', (2.05, -2.58)),
    'purple_cylinder': ('紫の円柱', (-1.25, 2.57)),
    'orange_sphere': ('オレンジの球', (-1.25, -2.65)),
    'obstacle': ('障害物', (0, 2.32)),
}


def transform_pose(element, point):
    """SDF pose: translation followed by ZYX Euler rotation of the local point."""
    pose = element.find('pose')
    if pose is None:
        return point
    if pose.get('relative_to'):
        raise ValueError('Named pose frames need explicit resolution before plotting')
    tx, ty, tz, roll, pitch, yaw = map(float, pose.text.split())
    x, y, z = point
    y, z = math.cos(roll)*y-math.sin(roll)*z, math.sin(roll)*y+math.cos(roll)*z
    x, z = math.cos(pitch)*x+math.sin(pitch)*z, -math.sin(pitch)*x+math.cos(pitch)*z
    x, y = math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y
    return x+tx, y+ty, z+tz


def world_point(model, link, collision, point):
    for element in (collision, link, model):
        point = transform_pose(element, point)
    return point


def hull(points):
    points = sorted(set(points))
    def cross(a, b, p):
        return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    def half(sequence):
        result = []
        for p in sequence:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result
    return half(points)[:-1] + half(reversed(points))[:-1]


def build_figure(save_assets=True):
    pdfmetrics.registerFont(UnicodeCIDFont(FONT))
    drawing = Drawing(1020, 560)
    drawing.add(Rect(0, 0, 1020, 560, fillColor=colors.white, strokeColor=None))
    provenance = {'description': 'Report-only orthographic top view, not a model input or measured trajectory.', 'worlds': []}

    def text(x, y, value, size=17, color=INK, anchor='middle'):
        drawing.add(String(x, y, value, fontName=FONT, fontSize=size, fillColor=color, textAnchor=anchor))

    for panel, (filename, title) in enumerate([
        ('visual_search.sdf', '標準ワールド（sim）'),
        ('hidden_search.sdf', '遮蔽ワールド（explore）'),
    ]):
        source = ROOT / 'src/pai_robot/worlds' / filename
        models = ET.parse(source).getroot().findall('./world/model')
        offset, bottom, size = 55+510*panel, 70, 430
        scale = size/6.8
        def xy(x, y):
            return offset+(x+3.4)*scale, bottom+(y+3.4)*scale
        drawing.add(Rect(offset, bottom, size, size, fillColor=colors.HexColor('#FAFCFD'), strokeColor=colors.HexColor('#CCD7DF'), strokeWidth=1))
        text(offset+size/2, 532, title, 21)
        for tick in range(-3, 4):
            gx, gy = xy(tick, tick)
            grid = colors.HexColor('#E2E8ED')
            drawing.add(Line(gx, bottom, gx, bottom+size, strokeColor=grid, strokeWidth=.7))
            drawing.add(Line(offset, gy, offset+size, gy, strokeColor=grid, strokeWidth=.7))
            text(gx, bottom-19, str(tick), 13, MUTED)
            text(offset-13, gy-4, str(tick), 13, MUTED)
        text(offset+size/2, bottom-42, 'X [m]', 15, MUTED)
        text(offset-35, bottom+size+4, 'Y [m]', 15, MUTED)
        robot = None
        record = {'source': str(source.relative_to(ROOT)), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'models': []}
        for model in models:
            name = model.get('name')
            if name == 'ground':
                continue
            pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
            record['models'].append({'name': name, 'pose': pose})
            if name == 'visual_bot':
                robot = pose
            for link in model.findall('link'):
                rgba = list(map(float, link.findtext('visual/material/ambient', '0.4 0.4 0.4 1').split()))
                fill = colors.Color(*rgba[:3])
                if name.startswith('wall_'):
                    fill = colors.HexColor('#BCC6CE')
                elif name in {'obstacle', 'occluding_screen', 'crate_w', 'crate_s', 'crate_e'}:
                    fill = colors.HexColor('#687783')
                elif name == 'visual_bot':
                    fill = INK
                for collision in link.findall('collision'):
                    geometry = collision.find('geometry')
                    if geometry.find('sphere') is not None:
                        radius = float(geometry.findtext('sphere/radius'))
                        center = world_point(model, link, collision, (0, 0, 0))
                        x, y = xy(*center[:2])
                        drawing.add(Circle(x, y, radius*scale, fillColor=fill, strokeColor=INK, strokeWidth=1))
                        continue
                    if geometry.find('box') is not None:
                        dimensions = list(map(float, geometry.findtext('box/size').split()))
                        vertices = list(itertools.product(*[(-d/2, d/2) for d in dimensions]))
                    elif geometry.find('cylinder') is not None:
                        radius = float(geometry.findtext('cylinder/radius'))
                        length = float(geometry.findtext('cylinder/length'))
                        vertices = [(radius*math.cos(i*math.tau/64), radius*math.sin(i*math.tau/64), z)
                                    for i in range(64) for z in (-length/2, length/2)]
                    else:
                        raise ValueError(f'Unsupported geometry: {name}')
                    points = hull([world_point(model, link, collision, p)[:2] for p in vertices])
                    drawing.add(Polygon([v for p in points for v in xy(*p)], fillColor=fill, strokeColor=INK, strokeWidth=1))
            if name in LABELS:
                label, position = LABELS[name]
                text(*xy(*position), label)

        assert robot is not None
        x, y, _, _, _, yaw = robot
        start = xy(x+.13*math.cos(yaw), y+.13*math.sin(yaw))
        tip = xy(x+.72*math.cos(yaw), y+.72*math.sin(yaw))
        drawing.add(Line(*start, *tip, strokeColor=INK, strokeWidth=2))
        head = [tip]
        for angle in (yaw+2.65, yaw-2.65):
            head.append((tip[0]+10*math.cos(angle), tip[1]+10*math.sin(angle)))
        drawing.add(Polygon([v for p in head for v in p], fillColor=INK, strokeColor=None))
        text(*xy(x, y-.55), 'ロボット初期位置', 16)
        text(*xy(x, y-.88), f'({x:g}, {y:g}) m', 14, MUTED)
        text(*xy(x+.65, y+.2), '+X', 13, MUTED)
        if panel == 1:
            screen = next(m for m in models if m.get('name') == 'occluding_screen')
            sx, sy, *_ = map(float, screen.findtext('pose').split())
            width, length, height = map(float, screen.findtext('link/collision/geometry/box/size').split())
            a, b = xy(sx, sy-length/2), xy(sx, -1.42)
            drawing.add(Line(*a, *b, strokeColor=MUTED, strokeWidth=1))
            text(*xy(sx, -1.75), f'遮蔽壁（高さ{height:g} m）', 16, MUTED)
        record['robot_initial_pose'] = robot
        provenance['worlds'].append(record)
    text(510, 5, '灰色：壁・障害物　　矢印：ロボットの初期向き　　目盛り間隔：1 m', 15, MUTED)

    if save_assets:
        assets = OUT / 'assets'
        assets.mkdir(exist_ok=True)
        svg_path = assets/'world_topview.svg'
        renderSVG.drawToFile(drawing, str(svg_path))
        # Read the shared font list without embedding any host-specific font path.
        node = shutil.which('node')
        if node is None:
            raise RuntimeError('Node.js is required. See output/pdf/BUILD.md.')
        font_list = subprocess.check_output([
            node, '--input-type=module', '-e',
            "import { minchoFonts } from './browser.mjs'; process.stdout.write(minchoFonts);",
        ], cwd=OUT, text=True)
        svg = svg_path.read_text().replace(f'font-family: {FONT};', f"font-family: {font_list.replace(chr(34), chr(39))};")
        svg_path.write_text(svg)
        subprocess.run([node, str(OUT/'render_figure.mjs'), str(svg_path), str(assets/'world_topview.png')], check=True)
        (OUT/'world_topview_sources.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2)+'\n')
    return drawing


if __name__ == '__main__':
    build_figure()
    print(OUT/'assets/world_topview.png')
