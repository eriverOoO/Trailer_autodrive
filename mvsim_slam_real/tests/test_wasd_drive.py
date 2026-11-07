"""Motor command tests use no real Arduino or hardware ports."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

spec=importlib.util.spec_from_file_location('wasd',Path(__file__).resolve().parents[1]/'tools/wasd_drive.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_combinations_and_protocol():
    assert m.command({'w','a'},60,3)==(-3,60,60)
    assert m.command({'s','d'},60,3)==(3,-60,-60)
    assert m.command({'w','s','a','d'},60,3)==(0,0,0)
    assert m.command({'a'},60,3)==(-3,0,0)
    assert m.packet(-7,-255,-255)==b's-7l-255r-255\n'
    assert len(m.packet(-7,-255,-255).rstrip(b'\n'))<15


@pytest.mark.parametrize('values',[(8,0,0),(0,256,0),(0,0,-256)])
def test_invalid_command_rejected(values):
    with pytest.raises(ValueError):m.packet(*values)


class FakeRoot:
    def __init__(self):self.jobs={};self.i=0
    def after(self,delay,fn):
        self.i+=1;self.jobs[self.i]=fn;return self.i
    def after_cancel(self,job):self.jobs.pop(job)


def test_space_stop_cannot_restart_on_autorepeat_until_release(monkeypatch):
    monkeypatch.setattr(m.time,'monotonic',lambda:10.)
    app=object.__new__(m.DriveWindow)
    app.keys={'w'};app.blocked=set();app.releases={};app.root=FakeRoot()
    app.ready_at=0.;app.fault=False;app.active=True;app.angle=-3
    sent=[];app.send=lambda *values:sent.append(values)
    app.stop()
    assert sent==[(-3,0,0)]
    app.press(NS(keysym='w'))
    assert not app.keys
    app.released('w')
    app.press(NS(keysym='w'))
    assert app.keys=={'w'}


def test_repeat_release_press_pair_is_cancelled(monkeypatch):
    monkeypatch.setattr(m.time,'monotonic',lambda:10.)
    app=object.__new__(m.DriveWindow)
    app.keys={'w'};app.blocked=set();app.releases={};app.root=FakeRoot()
    app.ready_at=0.;app.fault=False;app.active=True
    app.release(NS(keysym='w'))
    assert app.root.jobs
    app.press(NS(keysym='w'))
    assert not app.root.jobs and app.keys=={'w'}


def test_gui_serial_lifecycle_uses_only_pseudoterminal(monkeypatch):
    import os,pty,tkinter as tk
    from unittest.mock import Mock
    master,slave=pty.openpty()
    name=os.ttyname(slave)
    root=tk.Tk();root.withdraw()
    app=None
    monkeypatch.setattr(m.subprocess,'run',Mock(return_value=NS(returncode=1)))
    try:
        args=NS(dry_run=False,port=name,pwm=60,steer=3,steering_sign=-1,left_sign=1,right_sign=1)
        app=m.DriveWindow(root,args)
        app.ready_at=0.;app.active=True
        app.press(NS(keysym='w'));app.press(NS(keysym='a'))
        app.tick()
        app.stop()
        app.close()
        data=os.read(master,4096)
        assert data.startswith(b's0l0r0\n')
        assert b's-3l60r60\n' in data
        assert data.endswith(b's-3l0r0\n'*4)
    finally:
        if app is not None:app.close()
        else:root.destroy()
        os.close(master);os.close(slave)


def test_ros_wasd_held_keys_ready_loss_and_release_use_no_serial():
    import os,time,json,tkinter as tk
    import rclpy
    from std_msgs.msg import Bool,String
    from interfaces_pkg.msg import MotionCommand
    assert os.environ.get('ROS_DOMAIN_ID')=='223'
    root=tk.Tk();root.withdraw()
    args=NS(ros=True,dry_run=False,port=None,pwm=50,steer=3,steering_sign=-1,left_sign=1,right_sign=1)
    app=m.DriveWindow(root,args)
    source=rclpy.create_node('synthetic_only_wasd_bridge_fixture')
    ready=source.create_publisher(Bool,'/vehicle_viz_2d/ready',1)
    status=source.create_publisher(String,'/parking/manual_motor_status',1)
    received=[]
    source.create_subscription(MotionCommand,'/parking/manual_pwm',received.append,1)
    def spin(seconds,enabled):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            ready.publish(Bool(data=enabled))
            status.publish(String(data=json.dumps({'pwm':0,'steering':0,'reason':''})))
            app.tick();rclpy.spin_once(source,timeout_sec=.01)
    try:
        app.active=True
        spin(.3,False)
        app.press(NS(keysym='w'))
        assert not app.keys
        app.released('w')
        spin(.4,True)
        app.press(NS(keysym='w'));app.press(NS(keysym='a'))
        spin(.15,True)
        assert any((v.steering,v.left_speed,v.right_speed)==(-3,50,50) for v in received)
        app.released('w');app.released('a')
        spin(.1,True)
        assert received[-1].left_speed==received[-1].right_speed==0
        app.press(NS(keysym='w'));spin(.1,True)
        spin(.3,False)
        assert not app.keys
        assert received[-1].left_speed==received[-1].right_speed==0
        spin(.2,True)
        app.press(NS(keysym='w'))  # Held key stays blocked until released.
        assert not app.keys
        assert app.serial is None
    finally:
        source.destroy_node();app.close()
