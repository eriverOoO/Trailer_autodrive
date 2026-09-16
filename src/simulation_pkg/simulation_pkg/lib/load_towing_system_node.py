import copy
import math
import os
import random
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node


TRACTOR_NAME = 'ego_vehicle'
TRAILER_NAME = 'trailer_prius'
TRAILER_OFFSET = 4.8
FRONT_LEFT_WHEEL_LINK = 'front_left_wheel'
IN_START_TARGETS = (
    ((913, 482), (1.806422034, 14.699555461, 0.011641, 0.0, 0.0, -3.133789)),
    ((937, 482), (1.806422034, 15.793475868, 0.011641, 0.0, 0.0, -3.133795)),
    ((913, 507), (2.949727119, 14.699555461, 0.011641, 0.0, 0.0, -3.133797)),
    ((937, 507), (2.949727119, 15.793475868, 0.011641, 0.0, 0.0, -3.133797)),
)
PARKING_YAW = 0.011158653589793
SLOT_1_POSE = (7.151418196919, -7.725951808349, 0.011641, PARKING_YAW)
SLOT_4_POSE = (7.163876999707, 7.919384043882, 0.011641, PARKING_YAW)


def _remove_named(model, tag, name):
    for element in list(model.findall(tag)):
        if element.get('name') == name:
            model.remove(element)


def _remove_sensors(model, keep_link=None):
    for link in model.findall('link'):
        if link.get('name') == keep_link:
            continue
        for sensor in list(link.findall('sensor')):
            link.remove(sensor)


def _set_dynamics(axis, spring, damping, friction):
    dynamics = axis.find('dynamics')
    if dynamics is None:
        dynamics = ET.SubElement(axis, 'dynamics')
    values = {
        'spring_reference': 0.0,
        'spring_stiffness': spring,
        'damping': damping,
        'friction': friction,
    }
    for name, value in values.items():
        element = dynamics.find(name)
        if element is None:
            element = ET.SubElement(dynamics, name)
        element.text = str(value)


def _read_prius_model():
    package_share = get_package_share_directory('simulation_pkg')
    path = os.path.join(package_share, 'models', 'prius_hybrid', 'model.sdf')
    return ET.parse(path).getroot()


def _front_left_wheel_visual_center_local_xy(source_root):
    """Read the FL tire visual center in model coordinates from the source SDF."""
    model = source_root.find('model')
    link = next(
        (item for item in model.findall('link')
         if item.get('name') == FRONT_LEFT_WHEEL_LINK),
        None)
    if link is None:
        raise RuntimeError(
            f"Source Prius SDF has no '{FRONT_LEFT_WHEEL_LINK}' link")

    link_pose = link.find('pose')
    if link_pose is None or link_pose.text is None:
        raise RuntimeError(
            f"Source Prius SDF has no pose for '{FRONT_LEFT_WHEEL_LINK}'")
    link_values = [float(value) for value in link_pose.text.split()]
    if len(link_values) != 6:
        raise RuntimeError(
            f"Expected a 6-value pose for '{FRONT_LEFT_WHEEL_LINK}', "
            f'got {len(link_values)} values')

    visual = link.find('visual')
    visual_pose = None if visual is None else visual.find('pose')
    if visual_pose is None or visual_pose.text is None:
        raise RuntimeError(
            f"Source Prius SDF has no visual pose for '{FRONT_LEFT_WHEEL_LINK}'")
    visual_values = [float(value) for value in visual_pose.text.split()]
    if len(visual_values) != 6:
        raise RuntimeError(
            f"Expected a 6-value visual pose for '{FRONT_LEFT_WHEEL_LINK}', "
            f'got {len(visual_values)} values')

    link_x, link_y, _, _, _, link_yaw = link_values
    visual_x, visual_y, _, _, _, _ = visual_values
    center_x = (link_x + math.cos(link_yaw) * visual_x
                - math.sin(link_yaw) * visual_y)
    center_y = (link_y + math.sin(link_yaw) * visual_x
                + math.cos(link_yaw) * visual_y)
    return (link_x, link_y), (visual_x, visual_y), (center_x, center_y)


def _choose_random_ego_pose(source_root, start_index=0):
    """Choose one target and place the model so its FL tire visual center is there."""
    choices = tuple(enumerate(IN_START_TARGETS, start=1))
    if start_index not in range(5):
        raise ValueError('start_index must be 0 (random) or 1..4')
    point_index, (target_pixel, target) = (
        choices[start_index-1] if start_index else random.choice(choices))
    target_x, target_y, target_z, _, _, original_yaw = target
    corrected_yaw = math.atan2(
        math.sin(original_yaw + math.pi), math.cos(original_yaw + math.pi))
    link_local_xy, visual_local_xy, center_local_xy = (
        _front_left_wheel_visual_center_local_xy(source_root))
    fl_local_x, fl_local_y = center_local_xy

    dx = (math.cos(corrected_yaw) * fl_local_x
          - math.sin(corrected_yaw) * fl_local_y)
    dy = (math.sin(corrected_yaw) * fl_local_x
          + math.cos(corrected_yaw) * fl_local_y)
    ego_x = target_x - dx
    ego_y = target_y - dy
    ego_pose = (ego_x, ego_y, target_z, corrected_yaw)

    # This is the exact rigid-transform inverse used for spawning. It is
    # logged separately from the post-spawn Gazebo measurement.
    expected_x = ego_x + dx
    expected_y = ego_y + dy
    target_error = math.hypot(expected_x - target_x, expected_y - target_y)
    return (
        point_index, target_pixel, target, original_yaw, corrected_yaw,
        link_local_xy, visual_local_xy, center_local_xy, ego_pose,
        (expected_x, expected_y), target_error)


def _tractor_xml(source_root):
    root = copy.deepcopy(source_root)
    model = root.find('model')
    model.set('name', 'prius_hybrid_tractor')

    # The tractor retains the original front camera, Ackermann drive, odom,
    # wheel TF, joint states and p3d interfaces. Its old rear camera is removed
    # so /rear_camera/image_raw has exactly one publisher (the trailer).
    _remove_named(model, 'link', 'rear_camera_frame')
    _remove_named(model, 'joint', 'rear_camera_joint')

    # The source SDF uses the ROS 1 P3D remap key ~/out. On Humble the plugin
    # publishes on `odom`, so correct only the towing variant to restore the
    # intended /front_pose output without creating a second /odom publisher.
    p3d = next(plugin for plugin in model.findall('plugin') if plugin.get('name') == 'p3d')
    p3d.find('ros/remapping').text = 'odom:=front_pose'

    hitch = ET.fromstring(
        '''<plugin name="prius_physical_hitch" filename="libprius_hitch_joint_plugin.so">
             <trailer_model>trailer_prius</trailer_model>
             <parent_link>chassis</parent_link>
             <child_link>chassis</child_link>
             <parent_anchor>0 2.4 0.45</parent_anchor>
             <child_anchor>0 -2.4 0.45</child_anchor>
             <axis>0 0 1</axis>
             <lower>-0.7853981634</lower>
             <upper>0.7853981634</upper>
             <damping>10.0</damping>
           </plugin>''')
    model.append(hitch)
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


def _trailer_xml(source_root):
    root = copy.deepcopy(source_root)
    model = root.find('model')
    model.set('name', 'prius_hybrid_passive_trailer')

    # No front camera and no drive/odometry/joint-state/p3d/contact plugins.
    # The stock rear camera remains fixed to this model's chassis and retains
    # the established /rear_camera/image_raw interface.
    _remove_named(model, 'link', 'camera_frame')
    _remove_named(model, 'joint', 'camera_joint')
    _remove_named(model, 'link', 'base_link')
    _remove_named(model, 'joint', 'base_link_connection')
    _remove_sensors(model, keep_link='rear_camera_frame')
    for plugin in list(model.findall('plugin')):
        model.remove(plugin)

    # The stock model uses spherical, nearly isotropic tire collisions. Four
    # such contacts made the towed car scrub sideways instead of articulating.
    # Preserve the front wheel visuals and their real inertial mass, but fix
    # them to the chassis and remove only their ground collision. The rolling
    # rear Prius axle is the effective trailer axle; all body/rear-wheel
    # collisions remain active against the track and other vehicles.
    for joint_name in ('front_left_combined_joint', 'front_right_combined_joint'):
        joint = next(j for j in model.findall('joint') if j.get('name') == joint_name)
        joint.set('type', 'fixed')
        for axis_name in ('axis', 'axis2'):
            axis = joint.find(axis_name)
            if axis is not None:
                joint.remove(axis)
    for link_name in ('front_left_wheel', 'front_right_wheel'):
        link = next(link for link in model.findall('link') if link.get('name') == link_name)
        for collision in list(link.findall('collision')):
            link.remove(collision)
    for joint_name in ('rear_left_wheel_joint', 'rear_right_wheel_joint'):
        joint = next(j for j in model.findall('joint') if j.get('name') == joint_name)
        _set_dynamics(joint.find('axis'), spring=0.0, damping=0.4, friction=0.6)

    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


def _static_prius_xml(source_root):
    root = copy.deepcopy(source_root)
    model = root.find('model')
    model.set('name', 'prius_hybrid_static')
    static = model.find('static')
    if static is None:
        static = ET.SubElement(model, 'static')
    static.text = '1'
    _remove_sensors(model)
    for plugin in list(model.findall('plugin')):
        model.remove(plugin)
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


def _pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose


def _with_parking_lidar(xml, name):
    """Optional 360-degree rear-axle scan; only used by fused parking launch."""
    root = ET.fromstring(xml)
    chassis = root.find("model/link[@name='chassis']")
    mount_y = -2.45 if name == 'tractor' else 2.45
    chassis.append(ET.fromstring(f'''
      <sensor name="parking_{name}_scan" type="ray">
        <pose>0 {mount_y} 0.5 0 0 -1.5707963267948966</pose>
        <always_on>true</always_on><update_rate>15</update_rate>
        <ray><scan><horizontal><samples>720</samples><resolution>1</resolution>
          <min_angle>-3.141592653589793</min_angle><max_angle>3.141592653589793</max_angle>
        </horizontal></scan><range><min>0.15</min><max>20</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></ray>
        <plugin name="parking_{name}_lidar" filename="libgazebo_ros_ray_sensor.so">
          <ros><remapping>~/out:=/parking/{name}/scan</remapping></ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>{name}_rear_axle</frame_name>
        </plugin>
      </sensor>'''))
    return ET.tostring(root, encoding='unicode')


class TowingSystemLoader(Node):
    def __init__(self):
        super().__init__('towing_system_loader')
        self.start_index = self.declare_parameter('start_index', 0).value
        self.parking_lidar = self.declare_parameter('parking_lidar', False).value
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def spawn(self, name, xml, pose):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.initial_pose = pose
        request.reference_frame = 'world'
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            status = 'no response' if response is None else response.status_message
            raise RuntimeError(f"Failed to spawn {name}: {status}")
        self.get_logger().info(f"Spawned {name}: {response.status_message}")

    def load_all(self):
        self.get_logger().info('Waiting for Gazebo /spawn_entity service...')
        if not self.spawn_client.wait_for_service(timeout_sec=90.0):
            raise RuntimeError('Gazebo /spawn_entity service was not available after 90 s')

        source = _read_prius_model()
        tractor_xml = _tractor_xml(source)
        trailer_xml = _trailer_xml(source)
        if self.parking_lidar:
            tractor_xml = _with_parking_lidar(tractor_xml, 'tractor')
            trailer_xml = _with_parking_lidar(trailer_xml, 'trailer')
        static_xml = _static_prius_xml(source)

        (point_index, target_pixel, target, original_yaw, corrected_yaw,
         link_local_xy, visual_local_xy, center_local_xy, ego_pose,
         expected_fl_xy, target_error) = _choose_random_ego_pose(source, self.start_index)
        target_x, target_y, _, _, _, _ = target
        link_local_x, link_local_y = link_local_xy
        visual_local_x, visual_local_y = visual_local_xy
        center_local_x, center_local_y = center_local_xy
        ego_x, ego_y, ego_z, ego_yaw = ego_pose

        # Prius forward is model -Y, therefore its physical rear is local +Y.
        trailer_x = ego_x - TRAILER_OFFSET * math.sin(ego_yaw)
        trailer_y = ego_y + TRAILER_OFFSET * math.cos(ego_yaw)

        self.get_logger().info(
            f'[ParkingSpawn] selected IN point = {point_index}')
        self.get_logger().info(
            '[ParkingSpawn] target pixel = '
            f'({target_pixel[0]}, {target_pixel[1]})')
        self.get_logger().info(
            '[ParkingSpawn] target world xy = '
            f'({target_x:.6f}, {target_y:.6f})')
        self.get_logger().info(
            f'[ParkingSpawn] original yaw = {original_yaw:.6f}')
        self.get_logger().info(
            f'[ParkingSpawn] corrected yaw = {corrected_yaw:.6f}')
        self.get_logger().info(
            '[ParkingSpawn] FL wheel link local xy = '
            f'({link_local_x:.6f}, {link_local_y:.6f})')
        self.get_logger().info(
            '[ParkingSpawn] FL wheel visual local offset = '
            f'({visual_local_x:.6f}, {visual_local_y:.6f})')
        self.get_logger().info(
            '[ParkingSpawn] FL tire visual center model-local xy = '
            f'({center_local_x:.6f}, {center_local_y:.6f})')
        self.get_logger().info(
            '[ParkingSpawn] computed ego origin = '
            f'({ego_x:.6f}, {ego_y:.6f})')
        self.get_logger().info(
            '[ParkingSpawn] computed trailer origin = '
            f'({trailer_x:.6f}, {trailer_y:.6f})')
        self.get_logger().info(
            '[ParkingSpawn] expected FL tire visual center world xy = '
            f'({expected_fl_xy[0]:.6f}, {expected_fl_xy[1]:.6f})')
        self.get_logger().info(
            f'[ParkingSpawn] target error = {target_error:.12f} m')

        self.spawn(TRACTOR_NAME, tractor_xml, _pose(*ego_pose))
        self.spawn(
            TRAILER_NAME,
            trailer_xml,
            _pose(trailer_x, trailer_y, ego_z, ego_yaw))
        self.spawn('parked_prius_slot1', static_xml, _pose(*SLOT_1_POSE))
        self.spawn('parked_prius_slot4', static_xml, _pose(*SLOT_4_POSE))

        self.get_logger().info(
            'Towing system ready: tractor=(%.6f, %.6f, %.6f, %.6f), '
            'trailer=(%.6f, %.6f, %.6f, %.6f), slot1=%s, slot4=%s'
            % (ego_x, ego_y, ego_z, ego_yaw,
               trailer_x, trailer_y, ego_z, ego_yaw,
               SLOT_1_POSE, SLOT_4_POSE))


def main():
    rclpy.init()
    node = TowingSystemLoader()
    try:
        node.load_all()
    except Exception as exception:
        node.get_logger().fatal(str(exception))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
