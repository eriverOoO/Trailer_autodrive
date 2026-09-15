"""Extract simulation geometry/extrinsics; real hardware uses a measured JSON."""
import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .core import Geometry, State, trailer_pose


def rotation_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def simulation_calibration(sdf_path, loader_path, start_index):
    root = ET.parse(sdf_path).getroot().find('model')
    # Read constants as data, without importing the ROS loader or legacy pyc.
    constants = {}
    for node in ast.parse(Path(loader_path).read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign):
            try:
                constants[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError, AttributeError):
                pass
    if start_index not in range(1, 5):
        raise ValueError('Vision initialization needs deterministic start_index 1..4')
    pose = lambda name: np.array([float(x) for x in root.find(f"link[@name='{name}']/pose").text.split()])
    rear = (pose('rear_left_wheel')[:3]+pose('rear_right_wheel')[:3])/2
    front = (pose('front_left_wheel')[:3]+pose('front_right_wheel')[:3])/2
    rear[0], rear[2] = 0.0, 0.0
    extents = []
    for link in root.findall('link'):
        lp = np.fromstring(link.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        lr = rotation_rpy(*lp[3:])
        for collision in link.findall('collision'):
            cp = np.fromstring(collision.findtext('pose', '0 0 0 0 0 0'), sep=' ')
            center = lp[:3]+lr @ cp[:3]
            box = collision.findtext('geometry/box/size')
            sphere = collision.findtext('geometry/sphere/radius')
            if box:
                half = np.abs(lr @ rotation_rpy(*cp[3:])) @ (np.fromstring(box, sep=' ')/2)
            elif sphere:
                half = np.full(3, float(sphere))
            else:
                continue
            extents.extend([center-half, center+half])
    bounds = np.asarray(extents)
    low, high = bounds.min(axis=0), bounds.max(axis=0)
    g = Geometry(wheelbase=float(rear[1]-front[1]), hitch_offset=2.4-float(rear[1]),
                 trailer_axle=2.4+float(rear[1]), front=float(rear[1]-low[1]),
                 rear=float(high[1]-rear[1]), width=float(2*max(abs(low[0]), abs(high[0]))))
    _, target = constants['IN_START_TARGETS'][start_index-1]
    yaw_model = target[5]+math.pi
    fl = pose('front_left_wheel')
    visual = np.fromstring(root.find("link[@name='front_left_wheel']/visual/pose").text, sep=' ')
    local = fl[:3]+rotation_rpy(*fl[3:]) @ visual[:3]
    r_world_model = rotation_rpy(0, 0, yaw_model)
    origin = np.array(target[:3])-r_world_model @ local
    axle = origin+r_world_model @ rear
    state = State(float(axle[0]), float(axle[1]), yaw_model-math.pi/2, 0.0)
    body_from_model = rotation_rpy(0, 0, math.pi/2)
    sensor_from_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    cameras = {}
    for name, link in [('front', 'camera_frame'), ('rear', 'rear_camera_frame')]:
        p = pose(link)
        cameras[name] = {
            'rotation': (body_from_model @ rotation_rpy(*p[3:]) @ sensor_from_optical).tolist(),
            'translation': (body_from_model @ (p[:3]-rear)).tolist(),
            'initial_pose': [state.x, state.y, state.yaw] if name == 'front' else list(trailer_pose(state, g)),
        }
    return {'geometry': vars(g), 'initial_state': vars(state), 'cameras': cameras,
            'body_length': g.front+g.rear, 'rig_length': g.rig_length,
            'note': 'Collision envelope including spherical tire collisions; SI units.'}
