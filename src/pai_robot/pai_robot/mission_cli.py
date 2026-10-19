"""Resolve a natural-language mission, then publish it to the active controller."""
from __future__ import annotations
import argparse
import json
import sys
import time
from .ollama_client import OllamaClient

def main(argv=None):
    parser = argparse.ArgumentParser(description='Set the target of the active ROS controller')
    parser.add_argument('instruction', nargs='?', default='')
    parser.add_argument('--manual', choices=('red', 'blue'), help='Bypass language parsing only; image AI stays enabled')
    args = parser.parse_args(argv)
    if not args.manual and not args.instruction:
        parser.error('Supply an instruction or --manual red|blue')
    try:
        if args.manual:
            target, raw = args.manual, ''
        else:
            target, raw = OllamaClient().plan(args.instruction)
        mission = {'target_color': target, 'instruction': args.instruction,
                   'planning_mode': 'manual' if args.manual else 'llm', 'raw_plan': raw}
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        rclpy.init()
        node = Node('pai_mission_sender')
        pub = node.create_publisher(String, '/mission', QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        deadline = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No /mission subscriber. Start the simulation with controller:=true first.')
        msg = String()
        msg.data = json.dumps(mission, ensure_ascii=False)
        pub.publish(msg)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'Mission sent: target={target}; image VLM remains required in vision_mode=ai')
        node.destroy_node()
        rclpy.shutdown()
    except (ValueError, RuntimeError, OSError) as exc:
        print(f'Mission NOT sent: {exc}', file=sys.stderr)
        return 1
    return 0

if __name__ == '__main__':
    sys.exit(main())
