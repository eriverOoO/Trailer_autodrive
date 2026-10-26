#!/usr/bin/env python3
"""List USB serial identity/permissions/owners; never open a UART."""
import json
import os
import subprocess
from serial.tools import list_ports

out=[]
for p in list_ports.comports():
    if p.vid is None:continue
    owner=subprocess.run(['fuser',p.device],capture_output=True,text=True)
    out.append(dict(device=p.device,vid=f'{p.vid:04x}',pid=f'{p.pid:04x}',
                    manufacturer=p.manufacturer,description=p.description,
                    serial=p.serial_number,usb_location=p.location,
                    read_write=os.access(p.device,os.R_OK|os.W_OK),
                    fuser_exit=owner.returncode,owners=owner.stdout.strip(),
                    role='PHYSICAL_CONFIRMATION_REQUIRED'))
print(json.dumps(out,ensure_ascii=False,indent=2))
