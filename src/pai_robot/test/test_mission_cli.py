"""Mission receipt correlation, terminal outcomes and user-visible progress."""
from contextlib import redirect_stdout, redirect_stderr
import io
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from pai_robot import mission_cli
from pai_robot.mission_status import MissionReceipt, StatusDisplay, state_line, details
from pai_robot.agent import parse_mission


class Receipts(unittest.TestCase):
    def test_old_result_for_identical_instruction_is_not_receipt(self):
        receipt = MissionReceipt('new')
        self.assertFalse(receipt.receive({'mission_id':'old','state':'DONE','epoch':1,'instruction':'same'}))
        self.assertIsNone(receipt.latest)
        self.assertTrue(receipt.receive({'mission_id':'new','state':'THINKING','epoch':2}))
        self.assertFalse(receipt.receive({'mission_id':'old','state':'DONE','epoch':1}))
        self.assertFalse(receipt.replaced)
        receipt.receive({'mission_id':'replacement','state':'SETTLING','epoch':3})
        self.assertTrue(receipt.replaced)

    def test_optional_id_and_legacy_instruction_are_validated(self):
        value={'instruction':'赤い箱へ','mission_id':'a'*32}
        self.assertEqual(parse_mission(json.dumps(value)), '赤い箱へ')
        for bad in ('not-an-id', 5, None, 'x'*32):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                parse_mission(json.dumps({**value,'mission_id':bad}))

    def test_display_explains_stop_and_does_not_repeat_terminal_state(self):
        lines=[]
        display=StatusDisplay(lines.append)
        state={'mission_id':'x','state':'ERROR','step':0,'instruction':'左へ',
               'reason':'Wheel odometry clock moved backwards','log_dir':'/workspace/logs/run_x'}
        display.show(state,0)
        count=len(lines)
        display.show(state,20)
        self.assertEqual(len(lines),count)
        self.assertIn('時刻が逆戻り','\n'.join(lines))

    def test_motion_progress_has_direction_amount_and_odometry_label(self):
        line=state_line({'state':'EXECUTING','motion_progress':{'tool':'turn','goal':-90,'current':-30}})
        self.assertIn('右旋回 90.0°',line)
        self.assertIn('車輪推定 30.0°',line)

    def test_arrival_hold_and_automatic_completion_are_explained(self):
        text = '\n'.join(details({'candidate_assessment': {'candidate_matches_goal': True,
                                  'arrival_confirmation_pending': True, 'distance_estimate': None}}))
        self.assertIn('距離推定: 不明', text)
        self.assertIn('位置を保持して再観察中', text)
        self.assertIn('画像の到達条件を確認して完了', state_line(
            {'state': 'DONE', 'completion_source': 'camera_arrival_policy'}))
        self.assertIn('到達を再確認できず停止', state_line(
            {'state': 'STOPPED', 'last_tool_result': {'status': 'arrival_unconfirmed'}}))


class Commands(unittest.TestCase):
    def run_command(self, args, responses, subscribers=1, status=False, publishers=1):
        clock=[0.]
        emitted=[]
        receiver=[]
        published=[]
        class Pub:
            def get_subscription_count(self): return subscribers
            def publish(self,msg): published.append(json.loads(msg.data))
        class Node:
            def create_publisher(self,*args): return Pub()
            def count_publishers(self,topic): return publishers
            def destroy_node(self): pass
        def spin(node,timeout_sec):
            clock[0]+=1
            if responses:
                value=responses.pop(0)
                if value == 'matching':
                    value={'mission_id':published[-1]['mission_id'],'epoch':5,'state':'THINKING'}
                elif value == 'stopped':
                    value={'mission_id':published[-1]['mission_id'],'epoch':5,'state':'STOPPED','reason':'cannot proceed'}
                if value:
                    receiver[0](value)
        ros=SimpleNamespace(spin_once=spin,ok=lambda:True,shutdown=lambda:None)
        def connect(name,callback):
            receiver.append(callback)
            return ros,Node(),lambda:SimpleNamespace(data='')
        output=io.StringIO()
        with patch.object(mission_cli,'_connect',side_effect=connect), patch.object(mission_cli.time,'monotonic',side_effect=lambda:clock[0]), redirect_stdout(output), redirect_stderr(output):
            code=(mission_cli.status_main if status else mission_cli.main)(args)
        return code, output.getvalue(), published

    def test_send_waits_for_own_receipt_then_displays_model_stop(self):
        code,text,sent=self.run_command(['赤い箱へ'],[
            {'mission_id':'old','epoch':4,'state':'DONE','instruction':'赤い箱へ'},'matching','stopped'])
        self.assertEqual(code,2)
        self.assertEqual(len(sent),1)
        self.assertIn('受信を確認',text)
        self.assertIn('Qwenが判断中',text)
        self.assertIn('モデルの判断で停止',text)
        self.assertNotIn('[DONE]',text)

    def test_no_receipt_is_reported_as_unacknowledged_not_as_success(self):
        code,text,_=self.run_command(['赤い箱へ'],[])
        self.assertEqual(code,1)
        self.assertIn('受信確認がありません',text)

    def test_duplicate_executors_refuse_to_publish(self):
        code,text,sent=self.run_command(['赤い箱へ'],[],subscribers=2)
        self.assertEqual(code,1)
        self.assertEqual(sent,[])
        self.assertIn('実行側が複数',text)

    def test_send_only_still_waits_for_receipt(self):
        code,text,_=self.run_command(['赤い箱へ','--send-only'],['matching'])
        self.assertEqual(code,0)
        self.assertIn('受信を確認',text)
        self.assertIn('status --watch',text)

    def test_replaced_mission_ends_tracking(self):
        code,text,_=self.run_command(['赤い箱へ'],['matching',{'mission_id':'other','epoch':6,'state':'THINKING'}])
        self.assertEqual(code,3)
        self.assertIn('別のmission',text)

    def test_status_reports_existing_error_without_sending_mission(self):
        code,text,sent=self.run_command([], [{'state':'ERROR','reason':'Wheel odometry clock moved backwards'}],status=True)
        self.assertEqual(code,0)
        self.assertEqual(sent,[])
        self.assertIn('時刻が逆戻り',text)

    def test_watch_reports_duplicate_executors(self):
        code,text,_=self.run_command(['--watch'],[{'state':'IDLE'}],status=True,publishers=2)
        self.assertEqual(code,1)
        self.assertIn('実行側が複数',text)
        self.assertIn('状態配信を受信できません',text)
