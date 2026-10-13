

#!/usr/bin/env python3
"""TerraScout interactive ROS 2 mission tester; no external Python packages."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
NAMESPACE = os.environ.get('TERRASCOUT_NAMESPACE', 'terrascout1').strip('/')
BASE = ['ros2', 'run', 'fleet_common', 'send_mission.py', '--namespace', NAMESPACE]
LOG_DIR = ROOT / 'logs'


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


def start_mission(args):
    LOG_DIR.mkdir(exist_ok=True)
    logfile = LOG_DIR / f'mission_{time.strftime("%Y%m%d_%H%M%S")}.log'
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
    selected = list(waypoints.items())
    if not selected:
        print('No test waypoints are configured.')
        return
    print('\nMission waypoints:', ' -> '.join(name for name, _ in selected))
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
    start_mission(args)


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
7. Resume autonomy
8. Show test waypoints
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
                run_command(['ros2', 'service', 'call', f'/{NAMESPACE}/resume_autonomy',
                             'std_srvs/srv/Trigger', '{}'])
            elif choice == '8':
                for name, coord in waypoints.items():
                    print(f'{name}: {coord}')
            elif choice == '0':
                return 0
            else:
                print('Choose an option from 0 to 8.')
        except (KeyboardInterrupt, EOFError):
            print('\nExiting menu (any started mission client may continue).')
            return 0


if __name__ == '__main__':
    sys.exit(main())

