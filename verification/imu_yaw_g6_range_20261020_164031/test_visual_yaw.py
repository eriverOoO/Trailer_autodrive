"""Software-only sign/wrap regressions: publishes no ROS sensor inputs."""
import math
import pytest
from tractor_trailer_rviz_2d.core import YawTrack,rpy_quat

@pytest.mark.parametrize('start,end,sign,expected',[(10,30,-1,-20),(30,10,-1,20),(179,-179,-1,-2),(-179,179,-1,2),(10,30,1,20),(10,10,-1,0)])
def test_visual_only_sign_and_unwrap(start,end,sign,expected):
 t=YawTrack(sign=sign)
 q0=rpy_quat(0,0,math.radians(start));q1=rpy_quat(0,0,math.radians(end))
 assert t.push(10**9,1.,10**9,q=q0)
 initial=t.value
 assert t.push(1_100_000_000,1.1,1_100_000_000,q=q1)
 assert math.degrees(t.value-initial)==pytest.approx(expected)
 assert q1==rpy_quat(0,0,math.radians(end))  # native/public quaternion untouched

def test_sign_correction_is_only_in_tractor_diagnostic_config():
 from pathlib import Path
 import yaml
 base=Path('/home/venezuelachocolatemilk/mvsim_slam_real/visualization/tractor_trailer_rviz_2d/config')
 diag=yaml.safe_load((base/'tractor_only_diagnostic.yaml').read_text())
 full=yaml.safe_load((base/'live.yaml').read_text())
 assert diag['tractor']['sign']==-1
 assert diag['trailer']['sign']==1
 assert full['tractor']['sign']==1
 assert diag['tractor']['mounting_and_sign_confirmed'] is False
 assert diag['tractor_diagnostic']['extrinsics_confirmed'] is False
