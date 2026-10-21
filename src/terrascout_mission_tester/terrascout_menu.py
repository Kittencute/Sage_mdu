

#!/usr/bin/env python3
"""TerraScout interactive ROS 2 mission tester; no external Python packages."""
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parent
NAMESPACE = os.environ.get('TERRASCOUT_NAMESPACE', 'terrascout1').strip('/')
BASE = ['ros2', 'run', 'fleet_common', 'send_mission.py', '--namespace', NAMESPACE]
LOG_DIR = ROOT / 'logs'
RESULTS_DIR = ROOT / 'mission_results'
LOCAL_TIMEZONE = ZoneInfo('Europe/Stockholm')
MISSION_ROUTES = {
    '1': ('Mission 1', ['WP1', 'WP2', 'WP3', 'WP4', 'WP5', 'WP6']),
    '2': ('Mission 2', ['WP7', 'WP8', 'WP9', 'WP10', 'WP11', 'WP12']),
    '3': ('Mission 3', ['WP13', 'WP14', 'WP15', 'WP16', 'WP17', 'WP18']),
    '4': ('Mission 4', [
        'WP1', 'WP12', 'WP5', 'WP18', 'WP9', 'WP3', 'WP16', 'WP7', 'WP14', 'WP2',
        'WP20', 'WP11', 'WP4', 'WP17', 'WP8', 'WP19', 'WP6', 'WP13', 'WP10', 'WP15',
    ]),
}


def command_in_ros_environment(args):
    ros2 = shutil.which('ros2')
    if ros2:
        package = subprocess.run([ros2, 'pkg', 'prefix', 'fleet_common'],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                 check=False)
        if package.returncode == 0:
            return args
    make = shutil.which('make')
    if not make:
        raise FileNotFoundError('ROS 2 cannot find fleet_common and make is unavailable')
    return [make, 'exec', f'COMMAND={shlex.join(args)}']


def load_waypoints():
    with (ROOT / 'waypoints.json').open(encoding='utf-8') as f:
        raw = json.load(f)
    result = {}
    for name, pair in raw.items():
        lat, lon = map(float, pair)
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f'Invalid coordinates for {name}')
        result[name] = f'{lat:.8f},{lon:.8f}'
    return result


def choose_waypoints(waypoints, multiple=True):
    names = list(waypoints)
    print('\nAvailable test waypoints:')
    for i, name in enumerate(names, 1):
        print(f'  {i}. {name}: {waypoints[name]}')
    prompt = 'Waypoint numbers (e.g. 1 3 5; Enter to cancel): ' if multiple else 'Waypoint number (Enter to cancel): '
    tokens = input(prompt).replace(',', ' ').split()
    if not tokens:
        return []
    try:
        indices = [int(x) for x in tokens]
        if not multiple and len(indices) != 1:
            raise ValueError('Choose exactly one waypoint')
        if not all(1 <= i <= len(names) for i in indices):
            raise ValueError('Waypoint number out of range')
        if len(set(indices)) != len(indices):
            raise ValueError('Duplicate waypoints are not allowed')
    except ValueError as exc:
        print(f'Invalid selection: {exc}')
        return []
    selected = [(names[i - 1], waypoints[names[i - 1]]) for i in indices]
    print('Selected:', ' -> '.join(name for name, _ in selected))
    return selected


def run_command(args):
    try:
        command = command_in_ros_environment(args)
        print('\nRunning:', ' '.join(command))
        return subprocess.run(command, check=False).returncode
    except FileNotFoundError as exc:
        print(f'Cannot run ROS command: {exc}. Start the stack with make up first.')
        return 127
    except KeyboardInterrupt:
        print('\nCommand interrupted.')
        return 130


def start_mission(args, run_dir=None):
    log_dir = run_dir if run_dir is not None else LOG_DIR
    log_dir.mkdir(parents=True, exist_ok=True)
    logfile = log_dir / 'mission_client.log' if run_dir is not None else log_dir / f'mission_{time.strftime("%Y%m%d_%H%M%S")}.log'
    print('Mission output:', logfile)
    try:
        command = command_in_ros_environment(args)
        print('\nStarting:', ' '.join(command))
        with logfile.open('w', encoding='utf-8') as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
        print(f'Mission client started (PID {process.pid}). This does NOT confirm mission acceptance.')
        print('Check Foxglove and the log for acceptance/navigation status.')
    except FileNotFoundError as exc:
        print(f'Cannot start mission: {exc}. Start the stack with make up first.')


def send_mission(waypoints, mode):
    print('\nSelect mission route:')
    for key, (name, _) in MISSION_ROUTES.items():
        print(f'  {key}. {name}')
    mission_choice = input('Mission [1]: ').strip() or '1'
    if mission_choice not in MISSION_ROUTES:
        print('Cancelled: invalid mission')
        return
    mission_name, route = MISSION_ROUTES[mission_choice]
    missing = [name for name in route if name not in waypoints]
    if missing:
        print(f'Cannot send {mission_name}; missing waypoints: {", ".join(missing)}')
        return
    selected = [(name, waypoints[name]) for name in route]
    if not selected:
        print('No test waypoints are configured.')
        return
    print(f'\nSending {mission_name} ({mode}) with {len(selected)} waypoints.')
    print('\nFinal destination:')
    print('  1. Return to mission start (default)')
    print('  2. Saved Home')
    print('  3. No explicit return/home flag')
    choice = input('Select [1]: ').strip() or '1'
    if choice not in ('1', '2', '3'):
        print('Cancelled: invalid destination')
        return
    destination = {'1': ['--return'], '2': ['--home'], '3': []}[choice]
    args = BASE + [f'--{mode}'] + destination + [coord for _, coord in selected]
    started = datetime.now(LOCAL_TIMEZONE)
    run_dir = RESULTS_DIR / f"mission_{started.strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=False)
    route = {
        'mission_name': mission_name,
        'mission_preset': mission_choice,
        'ordering_mode': mode,
        'final_destination_option': {'1': 'return_to_start', '2': 'saved_home', '3': 'none'}[choice],
        'waypoints': [{'name': name, 'coordinates': list(map(float, coord.split(',')))}
                      for name, coord in selected],
    }
    (run_dir / 'waypoints.json').write_text(json.dumps(route, indent=2) + '\n', encoding='utf-8')
    metadata = {
        'bundle_created_at': started.isoformat(timespec='seconds'),
        'namespace': NAMESPACE,
        'mission_name': mission_name,
        'mission_preset': mission_choice,
        'ordering_mode': mode,
        'final_destination_option': route['final_destination_option'],
        'mission_client_status': 'started_unconfirmed',
        'mission_client_log': 'mission_client.log',
    }
    (run_dir / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    print('Mission result bundle:', run_dir)
    start_mission(args, run_dir)


def add_waypoint(waypoints):
    selected = choose_waypoints(waypoints, multiple=False)
    if not selected:
        return
    print('Add mode: 1. Ordered (append after unvisited)  2. Unordered (request reorder)')
    choice = input('Select [1]: ').strip() or '1'
    if choice not in ('1', '2'):
        print('Cancelled')
        return
    mode = 'ordered' if choice == '1' else 'unordered'
    run_command(BASE + ['--add', f'--{mode}', selected[0][1]])
    print('Note: the add command publishes a topic; verify it was received in mission server logs.')


def set_home(waypoints):
    selected = choose_waypoints(waypoints, multiple=False)
    if selected:
        run_command(BASE + ['--sethome', selected[0][1]])
        print('Saved Home may reset when the mission server restarts.')


def control_mission(request):
    return run_command([
        'ros2', 'service', 'call', f'/{NAMESPACE}/control_mission',
        'fleet_interfaces/srv/ControlMission', f'{{request: {request}}}',
    ])


def resume_autonomy():
    result = run_command([
        'ros2', 'service', 'call',
        f'/{NAMESPACE}/resume_autonomy', 'std_srvs/srv/Trigger', '{}',
    ])
    if result != 0:
        print('Resume request failed or timed out. Keep autonomy locked and inspect service logs.')
    return result


def inspect_mission_state():
    print(f'\nMission state for /{NAMESPACE}/mission_state (waiting up to 5 seconds):')
    args = [
        'ros2', 'topic', 'echo', '--once', '--timeout', '5.0',
        '--qos-durability', 'transient_local',
        f'/{NAMESPACE}/mission_state', 'fleet_interfaces/msg/MissionState',
    ]
    try:
        command = command_in_ros_environment(args)
        result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=8)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f'Could not get mission state: {exc}')
        return 1
    if result.returncode != 0 or not result.stdout.strip():
        detail = result.stderr.strip()
        print('No mission state received. Check that mission_server is running and publishing.')
        if detail:
            print(f'Detail: {detail.splitlines()[-1]}')
        return result.returncode or 1

    message = result.stdout

    def scalar(name, default='?'):
        match = re.search(rf'(?m)^{re.escape(name)}:\s*(.*)$', message)
        return match.group(1).strip().strip("'\"") if match else default

    def sequence(name):
        match = re.search(rf'(?ms)^{re.escape(name)}:\s*\n((?:\s+-\s+.*\n?)*)', message)
        if not match:
            return []
        return [item.strip() for item in re.findall(r'^\s+-\s+(.*)$', match.group(1), re.M)]

    mission_labels = {
        '0': 'IDLE', '1': 'RUNNING', '2': 'PAUSED',
        '3': 'FINISHED', '4': 'FAILED', '5': 'ABORTED',
    }
    command_labels = {
        '0': 'WAITING', '1': 'RUNNING', '2': 'PAUSED',
        '3': 'FINISHED', '4': 'FAILED', '5': 'ABORTED',
    }
    mission_status = scalar('mission_status')
    current_id = scalar('current_command_id')
    ready = scalar('ready').lower() == 'true'
    print(f"Mission: {scalar('mission_id')} — {mission_labels.get(mission_status, f'UNKNOWN ({mission_status})')}")
    print(f"Current command: {current_id}")
    print(f"Readiness: {'READY' if ready else 'NOT READY'}")
    if not ready and scalar('not_ready_reason', ''):
        print(f"  Reason: {scalar('not_ready_reason')}")
    if scalar('failure_reason', ''):
        print(f"Failure: {scalar('failure_reason')}")

    command_ids = sequence('command_ids')
    command_statuses = sequence('command_status')
    if command_ids:
        print('Commands:')
        for index, command_id in enumerate(command_ids):
            status = command_statuses[index] if index < len(command_statuses) else '?'
            marker = '  < current' if command_id == current_id else ''
            label = command_labels.get(status, f'UNKNOWN ({status})')
            print(f'  #{command_id}: {label}{marker}')
    return 0


def main():
    try:
        waypoints = load_waypoints()
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f'Cannot load waypoints.json: {exc}', file=sys.stderr)
        return 1
    while True:
        print(f'''\n========== TERRASCOUT MISSION TESTER ({NAMESPACE}) ==========
1. Send ORDERED mission
2. Send UNORDERED mission
3. Add waypoint to active mission
4. Clear / cancel mission
5. Go to saved Home
6. Set saved Home from a test waypoint
7. Resume autonomy (validates and resumes suspended mission)
8. Show test waypoints
9. Suspend mission
10. Show mission state
0. Exit''')
        try:
            choice = input('Select: ').strip()
            if choice == '1':
                send_mission(waypoints, 'ordered')
            elif choice == '2':
                send_mission(waypoints, 'unordered')
            elif choice == '3':
                add_waypoint(waypoints)
            elif choice == '4':
                run_command(BASE + ['--clear'])
            elif choice == '5':
                start_mission(BASE + ['--home'])
            elif choice == '6':
                set_home(waypoints)
            elif choice == '7':
                resume_autonomy()
            elif choice == '8':
                for name, coord in waypoints.items():
                    print(f'{name}: {coord}')
            elif choice == '9':
                control_mission(1)
            elif choice == '10':
                inspect_mission_state()
                print(f'Persistent snapshots are in /run/sage/{NAMESPACE}/ inside the robot container.')
                print('Files: mission_state.json and keepout_zones.json')
            elif choice == '0':
                return 0
            else:
                print('Choose an option from 0 to 10.')
        except (KeyboardInterrupt, EOFError):
            print('\nExiting menu (any started mission client may continue).')
            return 0


if __name__ == '__main__':
    sys.exit(main())

