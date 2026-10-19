"""Mocked ROS-adapter tests: these do not instantiate DDS or Gazebo."""
from concurrent.futures import Future
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
from pai_robot.ranges import Ranges

class FakeTwist:
    def __init__(self):
        self.linear=SimpleNamespace(x=0.,y=0.,z=0.)
        self.angular=SimpleNamespace(x=0.,y=0.,z=0.)
class FakeString:
    def __init__(self): self.data=''
class FakePublisher:
    def __init__(self): self.history=[]
    def publish(self, msg): self.history.append(msg)
class FakeNode:
    def __init__(self,name):
        self.parameters={}; self.publishers={}; self.subscriptions={}; self.services={}
        self.sim_ns=1_000_000_000
    def declare_parameter(self,name,value): self.parameters[name]=value
    def get_parameter(self,name): return SimpleNamespace(value=self.parameters[name])
    def create_publisher(self,typ,name,qos):
        pub=FakePublisher(); self.publishers[name]=pub; return pub
    def create_subscription(self,typ,name,callback,qos): self.subscriptions[name]=callback
    def create_service(self,typ,name,callback): self.services[name]=callback
    def create_timer(self,*args,**kwargs): pass
    def get_clock(self): return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.sim_ns))
    def get_logger(self): return SimpleNamespace(info=lambda s:None,warning=lambda s:None,error=lambda s:None)

mods={}
for name in ['rclpy','rclpy.node','rclpy.clock','rclpy.qos','sensor_msgs','sensor_msgs.msg','geometry_msgs','geometry_msgs.msg','std_msgs','std_msgs.msg','std_srvs','std_srvs.srv']:
    mods[name]=ModuleType(name)
mods['rclpy'].ok=lambda: True
mods['rclpy.node'].Node=FakeNode
mods['rclpy.clock'].Clock=lambda **kwargs:None
mods['rclpy.clock'].ClockType=SimpleNamespace(STEADY_TIME=1)
mods['rclpy.qos'].qos_profile_sensor_data=object()
mods['sensor_msgs.msg'].Image=object
mods['sensor_msgs.msg'].LaserScan=object
mods['geometry_msgs.msg'].Twist=FakeTwist
mods['std_msgs.msg'].String=FakeString
mods['std_srvs.srv'].Trigger=object
path=Path(__file__).resolve().parents[1]/'pai_robot/controller_node.py'
spec=importlib.util.spec_from_file_location('pai_robot._adapter_under_test',path)
module=importlib.util.module_from_spec(spec)
with patch.dict(sys.modules,mods): spec.loader.exec_module(module)

class Adapter(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        with patch.dict(os.environ,{'PAI_LOG_DIR':self.temp.name}): self.node=module.VisualRobot()
        self.node.ranges=Ranges(5,5,5,5,5,True)
        self.node.last_scan=time.monotonic()
        data=bytes([70,70,70,255])*(80*60)
        self.node.on_image(SimpleNamespace(data=data,width=80,height=60,step=320,encoding='bgra8'))
    def tearDown(self):
        self.node.close(); self.temp.cleanup()
    def mission(self,color='red'):
        m=FakeString(); m.data=json.dumps({'target_color':color}); self.node.on_mission(m)
    def tick(self):
        self.node.sim_ns+=100_000_000; self.node.tick()
    def assert_stopped(self):
        m=self.node.pub.history[-1]; self.assertEqual((m.linear.x,m.angular.z),(0.,0.))
    def test_real_topic_contract(self):
        self.assertEqual(set(self.node.subscriptions),{'/camera/image_raw','/scan','/mission'})
        self.assertIn('/cmd_vel_raw',self.node.publishers); self.assertIn('/halt',self.node.services)
    def test_idle_stops(self): self.tick(); self.assert_stopped()
    def test_missing_sensors_stops(self):
        self.mission(); self.node.last_scan=0; self.tick(); self.assert_stopped()
    def test_failed_vlm_never_approves(self):
        self.mission(); self.node.brain.state='VERIFY'
        f=Future(); f.set_exception(RuntimeError('offline error')); self.node.pending=f; self.node.pending_epoch=self.node.epoch
        self.tick(); self.assertEqual(self.node.brain.state,'ERROR'); self.assert_stopped()
    def test_old_mission_result_discarded(self):
        self.mission(); old=self.node.epoch
        self.mission('blue'); self.node.brain.state='VERIFY'
        f=Future(); f.set_result(({'red','blue'},'{"visible_colors":["red","blue"]}'))
        self.node.pending=f; self.node.pending_epoch=old
        self.node.image=None  # stop after draining pending result; do not start another job
        self.tick(); self.assertFalse(self.node.brain.approved); self.assertEqual(self.node.brain.state,'VERIFY')
    def test_pending_vlm_stops(self):
        self.mission(); self.node.brain.state='VERIFY'
        self.node.pending=Future(); self.node.pending_epoch=self.node.epoch; self.node.pending_started=time.monotonic()
        self.tick(); self.assert_stopped()
    def test_halt_invalidates_pending_result(self):
        self.mission(); old=self.node.epoch
        self.node.pending=Future(); self.node.pending_epoch=old; self.node.pending_started=time.monotonic()
        reply=self.node.on_halt(None,SimpleNamespace())
        self.assertTrue(reply.success); self.assertGreater(self.node.epoch,old)
        self.tick(); self.assert_stopped(); self.assertEqual(self.node.brain.state,'ERROR')
    def test_backwards_time_stops(self):
        self.mission(); self.node.last_sim=100
        self.tick(); self.assertEqual(self.node.brain.state,'ERROR'); self.assert_stopped()
    def test_invalid_new_mission_stops(self):
        self.mission(); m=FakeString(); m.data='{"target_color":"green"}'
        self.node.on_mission(m); self.tick(); self.assert_stopped()
    def test_bad_image_stops(self):
        self.mission(); self.node.on_image(SimpleNamespace(data=b'bad',width=80,height=60,step=320,encoding='rgb8'))
        self.tick(); self.assertEqual(self.node.brain.state,'ERROR'); self.assert_stopped()
