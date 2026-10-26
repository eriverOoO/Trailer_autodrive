#!/usr/bin/env python3
"""Install a reviewed G6 alias for one dedicated physical USB socket."""
import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
RULE = ROOT / 'verification/lidar_persistent_device_name/99-lidar-g6.rules'
DEST = Path('/etc/udev/rules.d/99-lidar-g6.rules')
ALIAS = Path('/dev/lidar_g6')
EXPECTED = {
    'ID_VENDOR_ID': '10c4', 'ID_MODEL_ID': 'ea60',
    'ID_SERIAL_SHORT': '0001', 'ID_USB_INTERFACE_NUM': '00',
    'ID_PATH': 'pci-0000:80:14.0-usb-0:7.1:1.0',
}


def properties(device):
    result = subprocess.check_output(
        ['udevadm', 'info', '-q', 'property', '-n', str(device)], text=True)
    return dict(line.split('=', 1) for line in result.splitlines() if '=' in line)


def rule_text():
    matches = ', '.join('ENV{' + k + '}=="' + v + '"' for k, v in EXPECTED.items())
    return ('# Dedicated G6 socket; same-ID IMU in this socket also matches.\n'
            'SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ' + matches +
            ', SYMLINK+="lidar_g6"\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true', help='Requires sudo; reserves current socket for G6')
    args = parser.parse_args()
    if RULE.read_text() != rule_text():
        raise RuntimeError('Candidate differs from expected dedicated-socket rule')
    devices = []
    for device in Path('/dev').glob('ttyUSB[0-9]*'):
        props = properties(device)
        if all(props.get(k) == value for k, value in EXPECTED.items()):
            devices.append((device, props))
    if len(devices) != 1:
        raise RuntimeError('Connect G6 to the originally checked USB socket; expected exactly one match')
    device, props = devices[0]
    occupied = subprocess.run(['fuser', str(device)], capture_output=True, text=True)
    if occupied.returncode != 1:
        raise RuntimeError('Device busy or ownership check failed: ' + occupied.stdout + occupied.stderr)
    for directory in ('/etc/udev/rules.d', '/run/udev/rules.d',
                      '/usr/local/lib/udev/rules.d', '/usr/lib/udev/rules.d'):
        for path in Path(directory).glob('*.rules'):
            if path == DEST:
                continue
            if 'lidar_g6' in path.read_text(errors='replace'):
                raise RuntimeError('Existing alias rule needs review: ' + str(path))
    if DEST.is_symlink() or (DEST.exists() and DEST.read_text() != rule_text()):
        raise RuntimeError('Different existing rule; refusing overwrite')
    if ALIAS.is_symlink():
        if ALIAS.resolve() != device.resolve():
            raise RuntimeError('Alias already points elsewhere')
    elif ALIAS.exists():
        raise RuntimeError('Alias path exists and is not a symlink')
    print('G6 dedicated socket:', EXPECTED['ID_PATH'])
    print('Current device:', device)
    print(rule_text(), end='')
    if not args.install:
        print('CHECK OK. No system changes. Use sudo with --install to register.')
        return
    if os.geteuid() != 0:
        raise RuntimeError('Run sudo /usr/bin/python3 tools/register_lidar_port.py --install in your terminal')
    if not DEST.exists():
        fd = os.open(DEST, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, 'w') as stream:
            stream.write(rule_text())
    subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
    subprocess.run(['udevadm', 'trigger', '--action=change', '/sys' + props['DEVPATH']], check=True)
    subprocess.run(['udevadm', 'settle', '--timeout=5'], check=True)
    if not ALIAS.is_symlink() or ALIAS.resolve() != device.resolve():
        raise RuntimeError('Rule installed but alias verification failed; inspect udev before continuing')
    print('REGISTERED:', ALIAS, '->', ALIAS.resolve())


if __name__ == '__main__':
    main()
