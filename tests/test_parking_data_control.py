"""Reference parity and pseudo UART only. Never opens a real device/camera."""
import importlib.util
import itertools
import marshal
from pathlib import Path
from types import SimpleNamespace as NS, CodeType, FunctionType

import pytest

spec=importlib.util.spec_from_file_location('parking_data_control',Path(__file__).parents[1]/'tools/parking_data_control.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_increment_hold_saturation_and_stopped_steering():
    control=m.DataControls(now=0.)
    for i in range(40):control.process({'a'},(i+1)*.11)
    assert control.packet()==b's-7l0r0\n'
    for i in range(40):control.process({'w'},5.+i*.11)
    assert control.packet()==b's-7l250r250\n'
    control.process(set(),10.)
    assert control.packet()==b's-7l250r250\n'
    for i in range(60):control.process({'s'},11.+i*.11)
    assert control.packet()==b's-7l-250r-250\n'
    control.process({'r'},20.)
    assert control.packet()==b's0l0r0\n'
    assert not control.exit
    control.process({'f'},21.)
    assert control.exit and control.packet()==b's0l0r0\n'


def test_matches_original_bytecode_for_all_key_combinations_and_timing():
    """Invoke only the reference method with fake keys/time/capture; no imports."""
    path=Path.home()/'ros2_parking/src/data_collection/data_collection_func_lib.cpython-310.pyc'
    if not path.exists():pytest.skip('local reference workspace unavailable')
    code=marshal.loads(path.read_bytes()[16:])
    def find(code,name):
        if code.co_name==name:return code
        for child in code.co_consts:
            if isinstance(child,CodeType):
                result=find(child,name)
                if result is not None:return result
    clock=NS(now=0.)
    keys=set()
    globals_={'time':NS(time=lambda:clock.now),'keyboard':NS(is_pressed=lambda k:k in keys),
              'print':lambda *args:None,'os':NS(path=NS(exists=lambda _:True)),
              'cv2':NS()}
    reference=FunctionType(find(code,'process'),globals_)
    # Camera branch uses a dummy capture that never produces an image.
    state=NS(steering=0,left_speed=0,right_speed=0,exit=False,max_steering=7,
             keyboard_sensing_period=.1,last_input_time=0.,
             data_collection_path='SYNTHETIC_ONLY',frame_num=0,cap=NS(read=lambda:(False,None)))
    control=m.DataControls(now=0.)
    cases=[set(combo) for n in range(8) for combo in itertools.combinations('wasdrcf',n)]
    for step,pressed in enumerate(cases*3):
        keys.clear();keys.update(pressed)
        clock.now+=.05 if step%3==0 else .11
        expected=reference(state)
        assert control.process(keys,clock.now)==expected
        assert (control.steering,control.left_speed,control.right_speed)==(state.steering,state.left_speed,state.right_speed)
        assert control.last_input_time==state.last_input_time
        assert control.packet()==f's{state.steering}l{state.left_speed}r{state.right_speed}\n'.encode()


def test_window_sends_stopped_steering_and_retained_speed_on_pseudo_uart():
    import os,pty,tkinter as tk
    import serial
    master,slave=pty.openpty()
    os.set_blocking(master,False)
    connection=serial.Serial(os.ttyname(slave),115200,timeout=1)
    keyboard=NS(pressed=lambda:set(),close=lambda:None)
    root=tk.Tk();root.withdraw()
    app=m.ParkingControlWindow(root,keyboard=keyboard,connection=connection)
    data=b''
    try:
        app.controls.process({'a'},app.controls.last_input_time+.11)
        app.send()
        assert app.controls.packet()==b's-1l0r0\n'
        app.controls.process({'w'},app.controls.last_input_time+.11)
        app.send()
        app.controls.process(set(),app.controls.last_input_time+.11)
        app.send()
        app.reset()
        app.close()
        data=os.read(master,65536)
        assert data.startswith(b's0l0r0\ns-1l0r0\n')
        assert b's-1l10r10\ns-1l10r10\n' in data
        assert data.endswith(b's0l0r0\n'*4)
        assert not connection.is_open
    finally:
        app.close();os.close(master);os.close(slave)


def test_x11_keyboard_can_poll_without_root():
    keyboard=m.X11Keyboard()
    try:
        assert all(keyboard.keycodes.values())
        assert keyboard.pressed()<=set('wasdrcf')
    finally:keyboard.close()


def test_launcher_closes_slam_when_control_window_exits(tmp_path):
    """Fake launch and GUI processes; /dev/null only, no serial/sensors."""
    import os,shutil,subprocess
    (tmp_path/'tools').mkdir()
    (tmp_path/'log').mkdir()
    (tmp_path/'install').mkdir()
    viz=tmp_path/'visualization/tractor_trailer_rviz_2d/install'
    viz.mkdir(parents=True)
    mockbin=tmp_path/'mockbin';mockbin.mkdir()
    (tmp_path/'install/setup.bash').write_text(f'export PATH="{mockbin}:$PATH"\n')
    (viz/'setup.bash').write_text('')
    fuser=mockbin/'fuser';fuser.write_text('#!/bin/bash\nexit 1\n');fuser.chmod(0o755)
    ros2=mockbin/'ros2'
    ros2.write_text('''#!/usr/bin/python3
import signal,time
from pathlib import Path
def stop(*args):
    Path('launch_stopped').write_text('TERM')
    raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
Path('launch_started').write_text('ok')
while True:time.sleep(.01)
''')
    ros2.chmod(0o755)
    (tmp_path/'tools/parking_data_control.py').write_text("import time; time.sleep(.3)\n")
    script=tmp_path/'tools/run_manual_slam_wasd.sh'
    shutil.copy(Path(__file__).parents[1]/'tools/run_manual_slam_wasd.sh',script)
    env=dict(os.environ,LIDAR_PORT='/dev/null',TRACTOR_IMU_PORT='/dev/null',
             TRAILER_IMU_PORT='/dev/null',ARDUINO_PORT='/dev/null')
    subprocess.run(['bash',str(script)],env=env,capture_output=True,check=True,timeout=10)
    assert (tmp_path/'launch_started').exists()
    assert (tmp_path/'launch_stopped').read_text()=='TERM'
