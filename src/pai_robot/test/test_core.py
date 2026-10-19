import io
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from pai_robot.images import to_bgra, png_from_bgra
from pai_robot.vision_tracking import find_target_blob, ColorBlob
from pai_robot.ranges import Ranges, summarize_scan
from pai_robot.behavior import VisualSearchBrain
from pai_robot.safety import guard_velocity
from pai_robot.ollama_client import parse_target, parse_colors

FREE = Ranges(5,5,5,5,5,True)
def blob(x=0.0,width=0.1,area=0.03):
    return ColorBlob(x,area,width,0.3,(1,1,20,30))
def verified():
    b = VisualSearchBrain('red'); b.state='VERIFY'; b.record_vision({'red'}); return b

def image(color='red',width=80,height=60):
    result=bytearray(bytes([80,80,80,255])*(width*height))
    rgb = (0,0,240,255) if color=='red' else (240,0,0,255)
    for y in range(10,50):
        for x in range(40,70): result[(y*width+x)*4:(y*width+x)*4+4]=bytes(rgb)
    return bytes(result)

class Images(unittest.TestCase):
    def test_rgb(self):
        self.assertEqual(to_bgra(b'\xff\x00\x00',1,1,3,'rgb8'),bytes([0,0,255,255]))
    def test_bgr(self):
        self.assertEqual(to_bgra(bytes([255,0,0]),1,1,3,'bgr8'),bytes([255,0,0,255]))
    def test_rgba(self):
        self.assertEqual(to_bgra(bytes([255,0,0,100]),1,1,4,'rgba8'),bytes([0,0,255,100]))
    def test_bgra(self):
        data=bytes([2,3,4,255]); self.assertEqual(to_bgra(data,1,1,4,'bgra8'),data)
    def test_padded_rows(self):
        self.assertEqual(to_bgra(bytes([255,0,0,9,9,0,0,255,9,9]),1,2,5,'rgb8'),bytes([0,0,255,255,255,0,0,255]))
    def test_bad_encoding(self):
        with self.assertRaises(ValueError): to_bgra(b'\x00',1,1,1,'mono8')
    def test_short_buffer(self):
        with self.assertRaises(ValueError): to_bgra(b'',10,10,30,'rgb8')
    def test_short_stride(self):
        with self.assertRaises(ValueError): to_bgra(bytes(30),3,2,4,'rgb8')
    def test_png_decodes(self):
        from PIL import Image
        im=Image.open(io.BytesIO(png_from_bgra(bytes([0,0,255,255]),1,1)))
        self.assertEqual(im.getpixel((0,0)),(255,0,0))
    def test_png_bad_geometry(self):
        with self.assertRaises(ValueError): png_from_bgra(b'123',1,1)
    def test_red_right(self):
        b=find_target_blob(image(),80,60,'red'); self.assertIsNotNone(b); self.assertGreater(b.center_x_norm,0)
    def test_blue_discrimination(self):
        self.assertIsNone(find_target_blob(image('blue'),80,60,'red'))
        self.assertIsNotNone(find_target_blob(image('blue'),80,60,'blue'))
    def test_gray_no_candidate(self):
        self.assertIsNone(find_target_blob(bytes([80,80,80,255])*4800,80,60,'red'))

class Scan(unittest.TestCase):
    def make(self,values): return summarize_scan(values,-math.pi,2*math.pi/359,0.03,6)
    def test_inf_is_free(self): self.assertEqual(self.make([math.inf]*360),Ranges(6,6,6,6,6,True))
    def test_all_nan_invalid(self): self.assertFalse(self.make([math.nan]*360).valid)
    def test_forward_obstacle(self):
        v=[5.0]*360; v[180]=0.25; r=self.make(v); self.assertEqual(r.front,0.25); self.assertEqual(r.left,5)
    def test_rear_obstacle(self):
        v=[5.0]*360; v[0]=0.2; self.assertEqual(self.make(v).rear,0.2)
    def test_empty(self): self.assertFalse(self.make([]).valid)
    def test_negative_increment(self): self.assertFalse(summarize_scan([1]*360,0,-0.01,0.03,6).valid)
    def test_short_field_not_full_scan(self): self.assertFalse(summarize_scan([1]*10,-0.1,0.02,0.03,6).valid)
    def test_close_saturation(self):
        v=[5.0]*360; v[180]=-math.inf; r=self.make(v); self.assertTrue(r.valid); self.assertEqual(r.front,0)
    def test_invalid_direction(self):
        v=[5.0]*360
        for i in range(165,195): v[i]=math.nan
        self.assertFalse(self.make(v).valid)

class Brain(unittest.TestCase):
    def test_no_candidate_rotate(self): self.assertGreater(VisualSearchBrain('red').step(None,False,FREE,0.1).angular,0)
    def test_no_approach_without_ai(self):
        b=VisualSearchBrain('red')
        for _ in range(20): a=b.step(blob(),True,FREE,0.1)
        self.assertEqual(b.state,'VERIFY'); self.assertEqual(a.linear,0)
    def test_same_frame_not_counted(self):
        b=VisualSearchBrain('red')
        for _ in range(30): b.step(blob(),True,FREE,0.1,new_image=False)
        self.assertEqual(b.state,'SEARCH')
    def test_settle_before_request(self):
        b=VisualSearchBrain('red')
        for _ in range(3): a=b.step(blob(),True,FREE,0.1)
        self.assertEqual(a.state,'SETTLE'); self.assertFalse(a.request_vision)
    def test_wrong_color_no_approval(self):
        b=VisualSearchBrain('red'); b.state='VERIFY'; b.record_vision({'blue'})
        self.assertFalse(b.approved); self.assertEqual(b.state,'SEARCH')
    def test_right_target_negative_yaw(self): self.assertLess(verified().step(blob(0.3),True,FREE,0.1).angular,0)
    def test_left_target_positive_yaw(self): self.assertGreater(verified().step(blob(-0.3),True,FREE,0.1).angular,0)
    def test_forward_centered(self): self.assertGreater(verified().step(blob(),True,FREE,0.1).linear,0)
    def test_lost_stops_then_reverifies(self):
        b=verified()
        for _ in range(5): a=b.step(None,False,FREE,0.1)
        self.assertEqual(a.linear,0); self.assertEqual(b.state,'SEARCH'); self.assertFalse(b.approved)
    def test_arrival_needs_size_and_range(self):
        b=verified(); a=b.step(blob(0,0.4,0.12),True,Ranges(.35,5,5,5,.35,True),.1)
        self.assertEqual(a.state,'DONE'); self.assertEqual(a.linear,0)
    def test_near_wall_not_tiny_target_done(self):
        b=verified(); a=b.step(blob(0,.1,.01),True,Ranges(.35,3,2,5,.35,True),.1)
        self.assertEqual(a.state,'AVOID')
    def test_retreat_blocked_behind(self):
        b=verified(); b.state='AVOID'; a=b.step(blob(),True,Ranges(.4,3,2,.2,.2,True),.1)
        self.assertEqual(a.linear,0)
    def test_invalid_ranges_stop(self): self.assertEqual(verified().step(blob(),True,Ranges.invalid(),.1).linear,0)
    def test_timeout_stops(self):
        b=VisualSearchBrain('red',mission_timeout=.2); a=b.step(None,False,FREE,.3); self.assertEqual(a.state,'ERROR')
    def test_verify_time_excluded_from_movement_budget(self):
        b=VisualSearchBrain('red',mission_timeout=.2); b.state='VERIFY'
        for _ in range(20): b.step(blob(),True,FREE,.2)
        self.assertEqual(b.state,'VERIFY')
    def test_failure_latches(self):
        b=verified(); b.fail('test'); self.assertEqual(b.step(blob(),True,FREE,1).state,'ERROR')
    def test_vision_wrong_state(self):
        with self.assertRaises(RuntimeError): VisualSearchBrain('red').record_vision({'red'})
    def test_simple_kinematic_convergence(self):
        # Synthetic geometry ONLY; not Gazebo physics or model inference.
        for goal_y in [-0.8,0.8]:
            b=VisualSearchBrain('red'); x,y,yaw=0.,0.,0.
            for _ in range(1500):
                dx,dy=2.1-x,goal_y-y; dist=math.hypot(dx,dy)
                angle=math.atan2(math.sin(math.atan2(dy,dx)-yaw),math.cos(math.atan2(dy,dx)-yaw))
                visible=abs(angle)<.60
                bl=blob(-math.tan(angle)/math.tan(.6),min(.9,.35/(2*max(.1,dist-.16)*math.tan(.6))),min(.7,.07/max(.05,dist*dist))) if visible else None
                front=dist-.27 if abs(angle)<.30 else 5.
                r=Ranges(front,5,5,5,min(front,5),True)
                a=b.step(bl,visible,r,.1)
                if a.request_vision: b.record_vision({'red'} if visible else set())
                x+=a.linear*math.cos(yaw)*.1; y+=a.linear*math.sin(yaw)*.1; yaw+=a.angular*.1
                if b.state in ('DONE','ERROR'): break
            self.assertEqual(b.state,'DONE', f'goal_y={goal_y}, state={b.state}')
            self.assertLess(dist,.8)

class Safety(unittest.TestCase):
    def call(self,v=.1,w=.2,r=FREE,**kw):
        opts=dict(command_age=.1,scan_age=.1,clock_age=.1); opts.update(kw)
        return guard_velocity(v,w,r,**opts)
    def test_passthrough(self): self.assertEqual(self.call(),(.1,.2,'ok'))
    def test_clamps(self): self.assertEqual(self.call(8,-8)[:2],(.12,-.7))
    def test_stale_cmd(self): self.assertEqual(self.call(command_age=.6)[:2],(0,0))
    def test_stale_scan(self): self.assertEqual(self.call(scan_age=2)[:2],(0,0))
    def test_paused_clock(self): self.assertEqual(self.call(clock_age=1)[:2],(0,0))
    def test_nan(self): self.assertEqual(self.call(v=math.nan)[:2],(0,0))
    def test_forward_stop(self): self.assertEqual(self.call(r=Ranges(.25,5,5,5,.25,True))[:2],(0,0))
    def test_reverse_stop(self): self.assertEqual(self.call(v=-.1,r=Ranges(5,5,5,.2,.2,True))[:2],(0,0))
    def test_near_side_stops_turn(self): self.assertEqual(self.call(v=0,r=Ranges(5,.1,5,5,.1,True))[:2],(0,0))

class OutputValidation(unittest.TestCase):
    def test_valid_plan(self): self.assertEqual(parse_target('{"target_color":"blue"}'),'blue')
    def test_unknown_plan(self):
        with self.assertRaises(ValueError): parse_target('{"target_color":"unknown"}')
    def test_array_plan(self):
        with self.assertRaises(ValueError): parse_target('["red"]')
    def test_extra_field(self):
        with self.assertRaises(ValueError): parse_target('{"target_color":"red","code":"run"}')
    def test_colors(self): self.assertEqual(parse_colors('{"visible_colors":["red","blue"]}'),{'red','blue'})
    def test_none_visible(self): self.assertEqual(parse_colors('{"visible_colors":[]}'),set())
    def test_duplicates(self):
        with self.assertRaises(ValueError): parse_colors('{"visible_colors":["red","red"]}')
    def test_wrong_label(self):
        with self.assertRaises(ValueError): parse_colors('{"visible_colors":["green"]}')

class Configuration(unittest.TestCase):
    def setUp(self):
        self.pkg=Path(__file__).resolve().parents[1]
        self.world=ET.parse(self.pkg/'worlds/visual_search.sdf').getroot()
    def test_no_external_assets(self):
        self.assertFalse(self.world.findall('.//include')); self.assertFalse(self.world.findall('.//uri'))
    def test_camera_lidar(self):
        self.assertEqual({s.attrib['type'] for s in self.world.findall('.//sensor')},{'camera','gpu_lidar'})
    def test_diffdrive(self):
        p=self.world.find(".//plugin[@name='ignition::gazebo::systems::DiffDrive']")
        self.assertEqual(p.findtext('topic'),'/cmd_vel'); self.assertEqual(p.findtext('wheel_separation'),'0.3')
    def test_bridge_directions(self):
        import yaml
        bridges=yaml.safe_load((self.pkg/'config/bridge.yaml').read_text())
        types={b['ros_topic_name']:b for b in bridges}
        self.assertEqual(types['/cmd_vel']['direction'],'ROS_TO_GZ')
        for k in ['/clock','/camera/image_raw','/scan','/odom']: self.assertEqual(types[k]['direction'],'GZ_TO_ROS')
        self.assertTrue(all(b['gz_type_name'].startswith('ignition.msgs.') for b in bridges))
    def test_package_xml(self):
        self.assertEqual(ET.parse(self.pkg/'package.xml').getroot().findtext('name'),'pai_robot')

if __name__=='__main__': unittest.main()
