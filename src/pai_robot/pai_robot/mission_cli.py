"""Send a mission, acknowledge it, and display progress through completion."""
import argparse
import json
import sys
import time
import uuid
from .agent import parse_mission
from .mission_status import MissionReceipt, StatusDisplay, TERMINAL


def _connect(name, receive):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node(name)
    def on_status(msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict) and isinstance(status.get('state'), str):
                receive(status)
        except (ValueError, TypeError):
            pass
    node.create_subscription(String, '/robot_state', on_status, 10)
    return rclpy, node, String


def _close(rclpy, node):
    if node is not None:
        node.destroy_node()
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('instruction', help='Navigation instruction, e.g. 赤い球を探して近くで止まって')
    parser.add_argument('--send-only', action='store_true', help='受信確認後に表示を終了し、車両は実行を継続')
    args = parser.parse_args(argv)
    node = rclpy = None
    sent = False
    try:
        mission_id = uuid.uuid4().hex
        receipt = MissionReceipt(mission_id)
        last_received = [0.0]
        def receive(status):
            if receipt.receive(status):
                last_received[0] = time.monotonic()
        payload = json.dumps({'instruction': args.instruction, 'mission_id': mission_id}, ensure_ascii=False)
        parse_mission(payload)
        rclpy, node, String = _connect('pai_mission_sender', receive)
        pub = node.create_publisher(String, '/mission', 10)
        deadline = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No /mission subscriber. Start ./run.sh sim first.')
        if pub.get_subscription_count() > 1:
            raise RuntimeError('実行側が複数あります。重複したsimを終了し、1つだけ起動してください。')
        msg = String()
        msg.data = payload
        pub.publish(msg)
        sent = True
        print(f'指令を送信しました（ID: {mission_id[:8]}）。実行側の受信確認を待っています。', flush=True)
        deadline = time.monotonic()+10
        while receipt.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if receipt.latest is None:
            raise RuntimeError('送信後の受信確認がありません。simを更新・再起動し、./run.sh statusで確認してください。')
        print('受信を確認しました。Ctrl+Cは表示のみ終了、車両停止は ./run.sh halt です。', flush=True)
        display = StatusDisplay(lambda text: print(text, flush=True))
        while True:
            if receipt.replaced:
                print('別のmissionに切り替わったため、この指令の追跡を終了します。', flush=True)
                return 3
            display.show(receipt.latest, time.monotonic())
            if args.send_only:
                print('車両の実行状況は ./run.sh status --watch で追跡できます。', flush=True)
                return 0
            state = receipt.latest['state']
            if state in TERMINAL:
                return 0 if state == 'DONE' else 2
            if time.monotonic()-last_received[0] > 10:
                raise RuntimeError('この指令の状態を10秒以上確認できません。実行側の再起動・切替を ./run.sh status で確認してください。')
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        print('\n表示を終了しました。車両の停止は ./run.sh halt で行ってください。', flush=True)
        return 130
    except (ValueError, RuntimeError, OSError) as exc:
        print('指令の追跡エラー:' if sent else '指令を送信できません:', exc, file=sys.stderr)
        return 1
    finally:
        _close(rclpy, node)


def status_main(argv=None):
    parser = argparse.ArgumentParser(description='現在の指令・動作・終了理由を表示します。')
    parser.add_argument('--watch', action='store_true', help='状態を継続表示。Ctrl+Cで表示を終了')
    args = parser.parse_args(argv)
    node = rclpy = None
    latest, received = [None], [time.monotonic()]
    def receive(status):
        latest[0], received[0] = status, time.monotonic()
    display = StatusDisplay(lambda text: print(text, flush=True))
    try:
        rclpy, node, _ = _connect('pai_mission_status', receive)
        warned = False
        while True:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if node.count_publishers('/robot_state') > 1 and not warned:
                print('警告: 実行側が複数あります。simを1つに整理してください。', flush=True)
                warned = True
            if latest[0] is not None:
                display.show(latest[0], now)
                if not args.watch:
                    return 0
            if now-received[0] > 10:
                raise RuntimeError('状態配信を受信できません。./run.sh sim の起動状態を確認してください。')
    except KeyboardInterrupt:
        return 0
    except (ValueError, RuntimeError, OSError) as exc:
        print('状態を取得できません:', exc, file=sys.stderr)
        return 1
    finally:
        _close(rclpy, node)
