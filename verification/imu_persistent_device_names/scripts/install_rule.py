#!/usr/bin/env python3
"""Install ONLY the reviewed dedicated-socket rule. Never overwrite existing rules."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
NAME='99-ebimu-tractor.rules'
DEST=Path('/etc/udev/rules.d')/NAME
RULE_DIRS=('/etc/udev/rules.d','/run/udev/rules.d','/usr/local/lib/udev/rules.d','/usr/lib/udev/rules.d')


def properties(device):
    text=subprocess.check_output(['udevadm','info','-q','property','-n',device],text=True)
    return dict(line.split('=',1) for line in text.splitlines() if '=' in line)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dedicated-port-confirmed',action='store_true',required=True)
    args=p.parse_args()
    selection=json.loads((ROOT/'device_selection.json').read_text())
    if selection['operator_confirmation']!='CURRENT_SOCKET_RESERVED_FOR_TRACTOR':
        raise RuntimeError('Socket reservation is pending. Do not install before confirming G6 uses another socket.')
    if os.geteuid()!=0:
        raise RuntimeError('Run with sudo in your normal terminal; never provide a password to the agent or a log.')
    for directory in RULE_DIRS:
        d=Path(directory)
        if (d/NAME).exists() or (d/NAME).is_symlink():
            raise RuntimeError('Existing same-name rule must not be overwritten: '+str(d/NAME))
        if not d.exists():continue
        for rule in d.glob('*.rules'):
            if rule.is_file() and 'imu_tractor' in rule.read_text(errors='replace'):
                raise RuntimeError('Existing tractor alias rule needs review: '+str(rule))
    alias=Path('/dev/imu_tractor')
    if alias.exists() or alias.is_symlink():raise RuntimeError('Alias already exists; refusing to replace it')
    matches=[]
    keys=('ID_VENDOR_ID','ID_MODEL_ID','ID_SERIAL_SHORT','ID_SERIAL','ID_USB_INTERFACE_NUM','ID_PATH')
    for device in Path('/dev').glob('ttyUSB[0-9]*'):
        props=properties(str(device))
        if all(props.get(k)==selection[k] for k in keys):matches.append((device,props))
    if len(matches)!=1:raise RuntimeError('Expected exactly one matching device in reserved socket')
    device,props=matches[0]
    if any(props.get(k)!=selection[k] for k in ('USEC_INITIALIZED','DEVPATH')):
        raise RuntimeError('Device was reconnected since real-packet verification; re-identify and refresh evidence first')
    occupied=subprocess.run(['fuser',str(device)],capture_output=True,text=True)
    if occupied.returncode!=1:raise RuntimeError('Device is occupied or ownership check failed: '+occupied.stdout+occupied.stderr)
    data=(ROOT/(NAME+'.candidate')).read_bytes()
    expected=(ROOT/'candidate_sha256.txt').read_text().strip()
    if hashlib.sha256(data).hexdigest()!=expected:raise RuntimeError('Reviewed rule content changed')
    # The candidate is only a matching rule and symlink assignment, with no
    # permissions, PROGRAM/RUN probe, USB EEPROM or sensor configuration writes.
    body='\n'.join(line for line in data.decode().splitlines() if line and not line.startswith('#'))
    if 'SYMLINK+="imu_tractor"' not in body or 'ENV{ID_PATH}=="'+selection['ID_PATH']+'"' not in body:
        raise RuntimeError('Unexpected candidate')
    print('Dedicated physical socket only; G6 in this socket would ALSO match.')
    print(data.decode(),flush=True)
    # O_EXCL also prevents an overwrite between the preflight and creation.
    fd=os.open(DEST,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o644)
    with os.fdopen(fd,'wb') as out:out.write(data)
    os.chmod(DEST,0o644)
    result=dict(installed=True,path=str(DEST),sha256=expected,device=str(device),
                id_path=selection['ID_PATH'],applied=False,alias_verified=False,
                scope='Dedicated socket; not port-independent identification')
    try:
        subprocess.run(['udevadm','control','--reload-rules'],check=True)
        syspath='/sys'+props['DEVPATH']
        test=subprocess.run(['udevadm','test','--action=change',syspath],capture_output=True,text=True)
        (ROOT/'evidence/udevadm_test_installed.txt').write_text(test.stdout+test.stderr)
        if test.returncode:raise RuntimeError('udevadm test failed; see diagnostic log')
        # Exactly this tty device, not a global USB trigger.
        subprocess.run(['udevadm','trigger','--action=change',syspath],check=True)
        subprocess.run(['udevadm','settle','--timeout=5'],check=True)
        result['applied']=True
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and not alias.is_symlink():time.sleep(.05)
        result['alias_verified']=alias.is_symlink() and alias.resolve()==device.resolve()
        result['alias_target']=str(alias.resolve()) if alias.is_symlink() else None
        if not result['alias_verified']:raise RuntimeError('Alias not pointing at verified device')
        st=device.stat()
        result['device_permissions']=oct(stat.S_IMODE(st.st_mode))
        result['device_gid']=st.st_gid
    except Exception as exc:
        result['error']=str(exc)
        # Do not delete or overwrite any unrelated rule/alias on an error.
        raise
    finally:
        (ROOT/'evidence/installation_result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
