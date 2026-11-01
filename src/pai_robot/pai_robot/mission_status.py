"""Human-readable ROS mission status; no inference or actuation."""
import math

TERMINAL = {'DONE', 'STOPPED', 'HALTED', 'ERROR'}
LABELS = {'IDLE': '指示待ち', 'SETTLING': '静止・撮影待ち', 'THINKING': 'Qwenが判断中',
          'EXECUTING': '動作実行中', 'DONE': 'モデルが到達を報告',
          'STOPPED': 'モデルの判断で停止', 'HALTED': '操作により停止', 'ERROR': 'エラーで停止'}


def state_line(status):
    state = status.get('state', '?')
    step = status.get('step', 0)
    label = LABELS.get(state, state)
    if state == 'DONE' and status.get('completion_source') == 'camera_arrival_policy':
        label = '画像の到達条件を確認して完了'
    if state == 'STOPPED' and (status.get('last_tool_result') or {}).get('status') == 'arrival_unconfirmed':
        label = '到達を再確認できず停止'
    prefix = f'[{state}] {label}'
    if status.get('instruction'):
        prefix += f' / 動作 {step} / 経過 {status.get("elapsed_wall_s", 0):.0f}秒'
    progress = status.get('motion_progress')
    if progress:
        goal, current = progress['goal'], progress['current']
        if progress['tool'] == 'move':
            prefix += f' / {"前進" if goal >= 0 else "後退"} {abs(goal):.2f} m、車輪推定 {abs(current):.2f} m'
        else:
            prefix += f' / {"左" if goal >= 0 else "右"}旋回 {abs(goal):.1f}°、車輪推定 {abs(current):.1f}°'
    elif state == 'THINKING':
        prefix += f' / 応答待ち {status.get("state_elapsed_s", 0):.0f}秒'
    return prefix


def details(status):
    lines = []
    if status.get('instruction'):
        lines.append('指令: ' + status['instruction'])
    if status.get('model'):
        lines.append('モデル: ' + status['model'])
    goal = status.get('fixed_goal')
    if goal:
        lines.append('固定した目標: ' + goal['description'])
        if goal['kind'] == 'approach':
            lines.append(f'到達条件: 車体前端から対象の手前まで {goal["max_clearance_m"]:g} m以内')
    assessment = status.get('candidate_assessment') or {}
    if assessment:
        lines.append('候補の色・形: ' + ('目標と一致' if assessment.get('candidate_matches_goal') else '不一致または未確認'))
        estimate = assessment.get('distance_estimate')
        if estimate:
            lines.append(f'画像からの距離推定: {estimate["clearance_m"]:.2f} m（平坦な床を仮定）')
        elif assessment.get('candidate_matches_goal'):
            lines.append('画像からの距離推定: 不明（対象と床の境界を確認できません）')
        if assessment.get('arrival_confirmation_pending'):
            lines.append('到達確認: 位置を保持して再観察中。前進・旋回は実行しません。')
    if status.get('reason'):
        reason = status['reason']
        if reason == 'Wheel odometry clock moved backwards':
            reason = '車輪オドメトリの時刻が逆戻りしました。シミュレーションの重複起動を確認してください。'
        lines.append('理由: ' + reason)
    observation = status.get('observation')
    if observation and observation != status.get('reason'):
        lines.append('直近のモデル観察: ' + observation)
    result = status.get('last_tool_result')
    if result and result.get('status') in ('rejected', 'motion_failed') and result.get('reason') != status.get('reason'):
        lines.append('直近の実行結果: ' + result['status'] + ' / ' + result.get('reason', ''))
    if status.get('state') == 'DONE' and not status.get('arrival_verified'):
        lines.append('画像による目標・距離チェック済み。独立した距離・接触の検証は行っていません。'
                     if status.get('camera_arrival_checks_passed') else
                     '完了はモデルの報告です。独立した到達・接触の検証は行っていません。')
    if status.get('log_dir'):
        lines.append('ログ: ' + status['log_dir'].replace('/workspace/', ''))
    return lines


class StatusDisplay:
    def __init__(self, emit=print):
        self.emit = emit
        self.key = None
        self.last_output = -math.inf

    def show(self, status, now):
        key = (status.get('mission_id'), status.get('epoch'), status.get('state'), status.get('step'))
        changed = key != self.key
        interval = 1 if status.get('state') == 'EXECUTING' else 5
        if not changed and (status.get('state') in TERMINAL or now-self.last_output < interval):
            return
        self.emit(state_line(status))
        if changed:
            for line in details(status):
                self.emit('  ' + line)
        self.key, self.last_output = key, now


class MissionReceipt:
    """Correlate repeated identical instructions without accepting an old result."""
    def __init__(self, mission_id):
        self.mission_id = mission_id
        self.latest = None
        self.replaced = False

    def receive(self, status):
        if status.get('mission_id') == self.mission_id:
            self.latest = status
            return True
        if (self.latest is not None and status.get('mission_id')
                and status.get('epoch', 0) > self.latest.get('epoch', 0)):
            self.replaced = True
        return False
