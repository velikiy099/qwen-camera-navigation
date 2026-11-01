"""Controller/watchdog behavior with mock ROS; no DDS or Gazebo instantiated."""
from concurrent.futures import Future
from dataclasses import asdict
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
from test_core import call


class FakeTwist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0., y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=0.)


class FakeString:
    def __init__(self):
        self.data = ''


class FakePublisher:
    def __init__(self):
        self.history = []

    def publish(self, msg):
        self.history.append(msg)


class FakeNode:
    def __init__(self, name):
        self.parameters, self.publishers, self.subscriptions, self.services = {}, {}, {}, {}
        self.sim_ns = 1_000_000_000

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, typ, name, qos):
        self.publishers[name] = FakePublisher()
        return self.publishers[name]

    def create_subscription(self, typ, name, callback, qos):
        self.subscriptions[name] = callback

    def create_service(self, typ, name, callback):
        self.services[name] = callback

    def create_timer(self, *args, **kwargs):
        pass

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.sim_ns))

    def get_logger(self):
        return SimpleNamespace(info=lambda s: None, warning=lambda s: None, error=lambda s: None)


class FakePool:
    def __init__(self, **kwargs):
        self.calls = []

    def submit(self, function, png, context):
        future = Future()
        self.calls.append((future, png, context))
        return future

    def shutdown(self, **kwargs):
        pass


mods = {name: ModuleType(name) for name in ['rclpy', 'rclpy.node', 'rclpy.clock', 'rclpy.qos',
        'sensor_msgs', 'sensor_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
        'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv']}
mods['rclpy'].ok = lambda: True
mods['rclpy.node'].Node = FakeNode
mods['rclpy.clock'].Clock = lambda **kwargs: None
mods['rclpy.clock'].ClockType = SimpleNamespace(STEADY_TIME=1)
mods['rclpy.qos'].qos_profile_sensor_data = object()
mods['sensor_msgs.msg'].Image = object
mods['nav_msgs.msg'].Odometry = object
mods['geometry_msgs.msg'].Twist = FakeTwist
mods['std_msgs.msg'].String = FakeString
mods['std_srvs.srv'].Trigger = object


def load(name):
    path = Path(__file__).resolve().parents[1] / 'pai_robot' / (name + '.py')
    spec = importlib.util.spec_from_file_location('pai_robot._test_' + name, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, mods):
        spec.loader.exec_module(module)
    return module


module, guard_module = load('controller_node'), load('guard_node')


def frame(stamp):
    ns = round(stamp * 1e9)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9)),
                           data=bytes([255, 0, 0]) * 4, width=2, height=2, step=6, encoding='rgb8')


def odometry(stamp, x=0., y=0., yaw=0., v=0., w=0.):
    return SimpleNamespace(header=frame(stamp).header,
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x,y=y,z=0.),
             orientation=SimpleNamespace(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=v),angular=SimpleNamespace(z=w))))


class Adapter(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.wall = 100.0
        self.pose = [0., 0., 0.]
        self.clock_patch = patch.object(module.time, 'monotonic', side_effect=lambda: self.wall)
        self.clock_patch.start()
        self.pool_patch = patch.object(module, 'ThreadPoolExecutor', FakePool)
        self.pool_patch.start()
        with patch.dict(os.environ, {'PAI_LOG_DIR': self.temp.name}):
            self.node = module.VisualRobot()
        self.node.tick()

    def tearDown(self):
        self.node.close()
        self.clock_patch.stop()
        self.pool_patch.stop()
        self.temp.cleanup()

    def mission(self, instruction='赤い箱に近づいて'):
        self.node.on_mission(SimpleNamespace(data=json.dumps({'instruction': instruction})))

    def tick(self, dt=.1, image=True, odom=True, wall_dt=None):
        self.wall += dt if wall_dt is None else wall_dt
        self.node.sim_ns += round(dt * 1e9)
        command = self.node.pub.history[-1]
        v, w = command.linear.x, command.angular.z
        self.pose[0] += v*math.cos(self.pose[2]+w*dt/2)*max(0,dt)
        self.pose[1] += v*math.sin(self.pose[2]+w*dt/2)*max(0,dt)
        self.pose[2] += w*max(0,dt)
        if odom:
            self.node.on_odom(odometry(self.node.sim_time(), *self.pose, v, w))
        if image:
            self.node.on_image(frame(self.node.sim_time()))
        self.node.tick()

    def thinking(self):
        self.mission()
        from pai_robot.navigation import Goal
        self.node.agent.lock_goal(Goal('approach', 'red box', 'red', 'box', 1.0))
        for _ in range(6):
            self.tick()
        self.assertEqual(self.node.agent.state, 'THINKING')
        self.assertIsNotNone(self.node.pending)

    def respond(self, action):
        self.node.pending.set_result((action, json.dumps(asdict(action))))
        self.tick()
        self.tick()

    def assert_stopped(self):
        msg = self.node.pub.history[-1]
        self.assertEqual((msg.linear.x, msg.angular.z), (0, 0))

    def test_rgb_agent_and_robot_odometry_topic_contract(self):
        self.assertEqual(set(self.node.subscriptions), {'/camera/image_raw', '/mission', '/odom'})
        self.assertIn('/cmd_vel_raw', self.node.publishers)
        self.assertIn('/halt', self.node.services)

    def test_status_identifies_repeated_identical_instructions(self):
        for mission_id in ('a'*32, 'b'*32):
            self.node.on_mission(SimpleNamespace(data=json.dumps({'instruction':'同じ指令','mission_id':mission_id})))
            status=json.loads(self.node.status_pub.history[-1].data)
            self.assertEqual(status['mission_id'],mission_id)
            self.assertEqual(status['state'],'SETTLING')

    def test_status_contains_motion_progress_and_observation(self):
        self.thinking()
        self.respond(call('turn',direction='right',angle_deg=30))
        status=json.loads(self.node.status_pub.history[-1].data)
        self.assertEqual(status['mission_id'],self.node.mission_id)
        self.assertAlmostEqual(status['motion_progress']['goal'],-30)
        self.assertEqual(status['motion_progress']['unit'],'deg')
        self.assertEqual(status['observation'],'Visible floor ahead')
        self.assertIn('elapsed_wall_s',status)
        self.assertIn('state_elapsed_s',status)

    def test_idle_stops(self):
        self.tick()
        self.assert_stopped()
        self.assertEqual(len(self.node.pool.calls), 0)

    def test_real_initial_stage_locks_goal_then_requests_navigation(self):
        from pai_robot.navigation import Goal
        self.mission()
        for _ in range(6):
            self.tick()
        self.assertEqual(self.node.pool.calls[-1][2]['task'], 'interpret_mission')
        self.respond(Goal('approach', '赤い箱', 'red', 'box', 1.0))
        self.assert_stopped()
        for _ in range(6):
            self.tick()
        context = self.node.pool.calls[-1][2]
        self.assertEqual(context['task'], 'navigate')
        self.assertEqual(context['fixed_goal']['color'], 'red')
        self.assertEqual(self.node.agent.actions, 0)

    def test_stationary_fresh_frame_sent_with_raw_instruction(self):
        self.thinking()
        future, png, context = self.node.pool.calls[0]
        self.assertTrue(png.startswith(b'\x89PNG'))
        self.assertEqual(context['instruction'], '赤い箱に近づいて')
        for _ in range(4):
            self.tick()
            self.assert_stopped()
        self.assertEqual(len(self.node.pool.calls), 1)

    def test_robot_selects_speed_and_stops_on_odometry(self):
        self.thinking()
        self.respond(call(distance_m=.06))
        self.assertGreater(self.node.pub.history[-1].linear.x, 0)
        for _ in range(100):
            self.tick()
            if self.node.agent.state != 'EXECUTING':
                break
        self.assert_stopped()
        self.assertEqual(self.node.agent.state, 'SETTLING')
        self.assertAlmostEqual(self.node.agent.history[-1]['odometry_distance_m'], .06, delta=.008)
        for _ in range(6):
            self.tick()
        self.assertEqual(len(self.node.pool.calls), 2)
        context = self.node.pool.calls[-1][2]
        self.assertEqual(context['recent_tool_results'][-1]['status'], 'completed')
        self.assertNotIn('odometry_distance_m', json.dumps(context))
        self.assertNotIn('odometry_angle_deg', json.dumps(context))

    def test_right_direction_becomes_negative_robot_yaw(self):
        self.thinking()
        self.respond(call('turn', direction='right', angle_deg=30))
        msg = self.node.pub.history[-1]
        self.assertEqual(msg.linear.x, 0)
        self.assertLess(msg.angular.z, 0)

    def test_failed_model_latches_stop(self):
        self.thinking()
        self.node.pending.set_exception(RuntimeError('offline'))
        self.tick()
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()
        self.tick()
        self.assertEqual(len(self.node.pool.calls), 1)

    def test_model_timeout(self):
        self.thinking()
        self.node.pending_started = self.wall - self.node.ai_timeout - 2
        self.tick()
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_completed_but_expired_response_cannot_move(self):
        self.thinking()
        self.node.pending_started = self.wall - self.node.ai_timeout - 2
        self.respond(call())
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_invalid_tool_response_is_logged_and_never_executed(self):
        self.thinking()
        self.node.pending.set_exception(module.InvalidToolResponse('bad tool arguments', '{"tool":"move"}'))
        self.tick()
        self.assertEqual(self.node.agent.state, 'SETTLING')
        self.assert_stopped()
        events = [json.loads(line) for line in (self.node.log_dir / 'events.jsonl').read_text().splitlines()]
        rejected = [row for row in events if row['event'] == 'rejected_response']
        self.assertEqual(rejected[-1]['raw'], '{"tool":"move"}')
        for _ in range(6):
            self.tick()
        self.assertEqual(len(self.node.pool.calls), 2)
        self.assertEqual(self.node.pool.calls[-1][2]['recent_tool_results'][-1]['reason'], 'bad tool arguments')

    def test_duplicate_odometry_does_not_refresh_watchdog(self):
        self.thinking()
        self.respond(call())
        duplicate = odometry(self.node.odom.stamp, *self.pose)
        for _ in range(7):
            self.node.on_odom(duplicate)
            self.tick(odom=False)
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_invalid_or_jumped_odometry_stops_motion(self):
        self.thinking()
        self.respond(call())
        self.node.on_odom(odometry(self.node.sim_time()+.05, 100,0,0))
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_nonfinite_odometry_stops_motion(self):
        self.thinking()
        self.respond(call())
        self.node.on_odom(odometry(self.node.sim_time()+.05, float('nan')))
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_halt_discards_late_motion(self):
        self.thinking()
        pending = self.node.pending
        self.node.on_halt(None, SimpleNamespace())
        pending.set_result((call(), '{}'))
        self.tick()
        self.assert_stopped()
        self.assertEqual(self.node.agent.state, 'HALTED')

    def test_new_mission_does_not_queue_or_accept_old_motion(self):
        self.thinking()
        old = self.node.pending
        self.mission('青い箱を探して')
        for _ in range(6):
            self.tick()
        self.assertEqual(len(self.node.pool.calls), 1)
        self.assert_stopped()
        old.set_result((call(), '{}'))
        self.tick()
        self.assert_stopped()
        self.assertEqual(self.node.agent.actions, 0)
        self.assertEqual(len(self.node.pool.calls), 2)
        self.assertEqual(self.node.pool.calls[-1][2]['instruction'], '青い箱を探して')

    def test_failed_old_response_does_not_fail_new_mission(self):
        self.thinking()
        old = self.node.pending
        self.mission('青い箱を探して')
        old.set_exception(RuntimeError('old failure'))
        self.tick()
        self.assertEqual(self.node.agent.state, 'SETTLING')

    def test_duplicate_frames_do_not_refresh_watchdog(self):
        self.thinking()
        self.respond(call())
        duplicate = frame(self.node.image_time)
        for _ in range(8):
            self.node.on_image(duplicate)
            self.tick(image=False)
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_missing_initial_camera_waits_then_stops(self):
        self.mission()
        self.tick(image=False)
        self.assert_stopped()
        self.tick(image=False, wall_dt=11)
        self.assertEqual(self.node.agent.state, 'ERROR')

    def test_paused_clock_aborts_motion(self):
        self.thinking()
        self.respond(call())
        self.tick(dt=0, image=False, wall_dt=1)
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_backwards_clock_aborts_and_discards_inference(self):
        self.thinking()
        future = self.node.pending
        self.tick(dt=-.5, image=False, wall_dt=.1)
        future.set_result((call(), '{}'))
        self.tick()
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_large_simulation_jump_aborts_motion(self):
        self.thinking()
        self.respond(call())
        self.tick(dt=1)
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_bad_camera_stops_immediately(self):
        self.thinking()
        self.respond(call())
        bad = frame(self.node.sim_time() + .1)
        bad.data = b''
        self.node.on_image(bad)
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_invalid_new_mission_stops_active_motion(self):
        self.thinking()
        self.respond(call())
        self.node.on_mission(SimpleNamespace(data='{"instruction":null}'))
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_wall_time_budget_stops(self):
        self.thinking()
        self.node.max_wall = .5
        self.tick()
        self.assertEqual(self.node.agent.state, 'ERROR')
        self.assert_stopped()

    def test_finish_does_not_claim_verified_arrival(self):
        self.thinking()
        from test_core import arrival_png
        self.node.agent.search.image_size = (320, 240)
        self.node.agent.search.last_png = arrival_png()
        self.node.agent.search.frame_id = 2
        self.node.agent.previous_near_frame = 1
        self.respond(call('finish'))
        self.assert_stopped()
        status = json.loads(self.node.status_pub.history[-1].data)
        self.assertEqual(status['state'], 'DONE')
        self.assertFalse(status['arrival_reported_by_model'])
        self.assertEqual(status['completion_source'], 'camera_arrival_policy')
        self.assertTrue(status['camera_arrival_checks_passed'])
        self.assertFalse(status['arrival_verified'])
        self.assertEqual(status['motion_feedback'], 'wheel_odometry')
        self.assertFalse(status['ground_truth_used'])


class IndependentGuard(unittest.TestCase):
    def setUp(self):
        self.wall = 100.
        self.clock_patch = patch.object(guard_module.time, 'monotonic', side_effect=lambda: self.wall)
        self.clock_patch.start()
        self.node = guard_module.VelocityGuard()
        self.node.tick()
        self.node.sim_ns += 100_000_000
        self.node.on_image(frame(1.1))
        self.node.on_odom(odometry(1.1))
        msg = FakeTwist()
        msg.linear.x = .1
        self.node.on_command(msg)
        self.node.tick()

    def tearDown(self):
        self.clock_patch.stop()

    def test_valid_motion_passes_without_obstacle_claim(self):
        self.assertEqual(self.node.pub.history[-1].linear.x, .1)
        self.assertFalse(json.loads(self.node.status.history[-1].data)['obstacle_detection'])
        self.assertEqual(set(self.node.subscriptions), {'/cmd_vel_raw', '/camera/image_raw', '/odom'})

    def test_dead_controller_stops_even_with_fresh_camera_and_clock(self):
        self.wall += .6
        self.node.sim_ns += 600_000_000
        self.node.on_image(frame(1.7))
        self.node.on_odom(odometry(1.7))
        self.node.tick()
        self.assertEqual(self.node.pub.history[-1].linear.x, 0)

    def test_duplicate_camera_and_fresh_commands_still_stop(self):
        self.wall += 2
        self.node.sim_ns += 2_000_000_000
        self.node.on_image(frame(1.1))
        self.node.on_odom(odometry(3.1))
        msg = FakeTwist()
        msg.linear.x = .1
        self.node.on_command(msg)
        self.node.tick()
        self.assertEqual(self.node.pub.history[-1].linear.x, 0)

    def test_clock_rollback_drops_old_command(self):
        self.node.sim_ns = 100_000_000
        self.node.tick()
        self.assertEqual(self.node.pub.history[-1].linear.x, 0)

    def test_missing_odometry_stops_with_fresh_camera_clock_and_commands(self):
        self.wall += .6
        self.node.sim_ns += 600_000_000
        self.node.on_image(frame(1.7))
        msg = FakeTwist()
        msg.linear.x = .1
        self.node.on_command(msg)
        self.node.tick()
        self.assertEqual(self.node.pub.history[-1].linear.x, 0)
        self.assertEqual(json.loads(self.node.status.history[-1].data)['reason'], 'odometry_stale')
