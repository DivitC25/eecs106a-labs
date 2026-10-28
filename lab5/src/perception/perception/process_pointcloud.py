import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker, MarkerArray

# The table height next to the cube comes from the points in this ring around it
# (just outside the cube, so none of the cube's own points get in)
TABLE_RING_MIN = 0.06  # m
TABLE_RING_MAX = 0.15  # m

class RealSensePCSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_pc_subscriber')
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value
        self.max_y = float(self.declare_parameter('max_y', 0.79).value)

        self.min_z = float(self.declare_parameter('min_z', -0.18).value)
        self.max_z = float(self.declare_parameter('max_z', -0.08).value)

        self.bounds_pub = self.create_publisher(MarkerArray, '/filter_planes', 1)

        # Part 5: the cube is black, so its points should be dark (HSV value at most this)
        self.cube_max_v = int(self.declare_parameter('cube_max_v', 80).value)

        # Part 5: HSV range for the blue tape (in OpenCV, hue goes from 0 to 179).
        # Use `ros2 run perception hsv_tuner` to find good values for your tape.
        self.tape_h_min = int(self.declare_parameter('tape_h_min', 100).value)
        self.tape_h_max = int(self.declare_parameter('tape_h_max', 130).value)
        self.tape_s_min = int(self.declare_parameter('tape_s_min', 120).value)
        self.tape_v_min = int(self.declare_parameter('tape_v_min', 50).value)
        self.tape_min_points = int(self.declare_parameter('tape_min_points', 30).value)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose', 1)
        self.cube_height_pub = self.create_publisher(Float32, '/cube_height', 1)
        self.filtered_points_pub = self.create_publisher(PointCloud2, '/filtered_points', 1)
        self.tape_pose_pub = self.create_publisher(PointStamped, '/tape_pose', 1)
        self.tape_points_pub = self.create_publisher(PointCloud2, '/tape_points', 1)

        self.get_logger().info("Subscribed to PointCloud2 topic and marker publisher ready")

    def pointcloud_callback(self, msg: PointCloud2):

        # Transform the pointcloud from its original frame to base_link
        # Lookup Transform and use library function to transform cloud

        # Filter points between z coords between min_z and max_z and max_y
        # Call the numpy array filtered_points

        source_frame = 'camera_depth_optical_frame'  
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, source_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {source_frame} to {self.target_frame}: {ex}')
            return

        transformed_cloud = do_transform_cloud(msg, tf)


        self.publish_filter_planes(transformed_cloud.header)
        
        raw_points = pc2.read_points(
            transformed_cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        points_base = np.column_stack(
                (raw_points['x'], raw_points['y'], raw_points['z'])
            ).astype(np.float32, copy=False)

        # Part 5: HSV color of each point, in the same order as points_base.
        # This stays None until you do point_colors_hsv, so the parts before still work.
        hsv = self.point_colors_hsv(msg)
        if hsv is not None and len(hsv) != len(points_base):
            self.get_logger().warn('error in detecting hsv')
            hsv = None
        if hsv is not None:
            self.find_tape(points_base, hsv, transformed_cloud.header)

        mask = (
            (points_base[:, 2] >= self.min_z) &
            (points_base[:, 2] <= self.max_z) &
            (points_base[:, 1] <= self.max_y)
        )
        if hsv is not None:
            mask &= hsv[:, 2] <= self.cube_max_v
        filtered_points = points_base[mask]
        # filtered_points = points_base

        if filtered_points.size == 0:
            self.get_logger().warn(
                f'No points after filters: z in [{self.min_z:.3f}, {self.max_z:.3f}] m, y <= {self.max_y:.3f} m'
            )
            return

        filtered_cloud = pc2.create_cloud_xyz32(
            transformed_cloud.header,
            filtered_points.tolist(),
        )
        self.filtered_points_pub.publish(filtered_cloud)

        cube_x = float(np.median(filtered_points[:, 0]))
        cube_y = float(np.median(filtered_points[:, 1]))

        
        cube_top = float(np.percentile(filtered_points[:, 2], 95))

        dist_xy = np.hypot(points_base[:, 0] - cube_x, points_base[:, 1] - cube_y)
        ring_points = points_base[(dist_xy >= TABLE_RING_MIN) & (dist_xy <= TABLE_RING_MAX)]
        if ring_points.size == 0:
            self.get_logger().warn('No table ring points found around the cube, skipping')
            return
        table_z = float(np.median(ring_points[:, 2]))

        cube_height = cube_top - table_z
        cube_z = table_z + cube_height / 2  # middle of the cube
        if cube_top > self.max_z - 0.005:
            # the top of the cube got cut off by max_z, so the cube looks shorter than it is
            self.get_logger().warn(f'Top of the cube is at max_z ({self.max_z:.3f}), raise max_z '
                                   'so the whole cube gets through the filter', throttle_duration_sec=2.0)
        self.cube_height_pub.publish(Float32(data=cube_height))

        cube_pose = PointStamped()
        cube_pose.header = transformed_cloud.header
        cube_pose.point.x = cube_x
        cube_pose.point.y = cube_y
        cube_pose.point.z = cube_z

        self.cube_pose_pub.publish(cube_pose)

    def point_colors_hsv(self, msg):
        # The RealSense packs each point's color into one float field called 'rgb'.
        # Returns an (N, 3) uint8 array with H, S, V for each point, in the same order
        # as points_base (only the points with a valid x, y, z are kept, like skip_nans).
        if 'rgb' not in [f.name for f in msg.fields]:
            return None
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=False)
        valid = ~(np.isnan(pts['x']) | np.isnan(pts['y']) | np.isnan(pts['z']))
        packed = np.ascontiguousarray(pts['rgb'][valid], dtype=np.float32).view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=1).astype(np.uint8)

        # cvtColor expects an image, so reshape to (N, 1, 3) and back. These colors are
        # in RGB order, not the BGR order OpenCV usually uses.
        hsv = cv2.cvtColor(rgb.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)
        return hsv

    def find_tape(self, points_base, hsv, header):
        h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]
        color_mask = (
            (h >= self.tape_h_min) & (h <= self.tape_h_max) &
            (s >= self.tape_s_min) & (v >= self.tape_v_min)
        )
        geom_mask = (points_base[:, 2] <= self.max_z) & (points_base[:, 1] <= self.max_y)
        tape_points = points_base[color_mask & geom_mask]

        if len(tape_points) < self.tape_min_points:
            return

        self.tape_points_pub.publish(pc2.create_cloud_xyz32(header, tape_points.tolist()))

        centroid = tape_points.mean(axis=0)
        tape_pose = PointStamped()
        tape_pose.header = header
        tape_pose.point.x = float(centroid[0])
        tape_pose.point.y = float(centroid[1])
        tape_pose.point.z = float(centroid[2])
        self.tape_pose_pub.publish(tape_pose)

    def publish_filter_planes(self, header):
        marker_array = MarkerArray()

        def create_plane(marker_id, x, y, z, scale_x, scale_y, scale_z, r, g, b):
            m = Marker()
            m.header = header
            m.ns = "filter_bounds"
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
           
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            
            m.scale.x = float(scale_x)
            m.scale.y = float(scale_y)
            m.scale.z = float(scale_z)
            
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 0.4
            
            return m

        
        span = 2.0
        thickness = 0.002

        # minz
        marker_array.markers.append(
            create_plane(0, 0.0, 0.0, self.min_z, span, span, thickness, 1.0, 0.0, 0.0))
        
        # maxz
        marker_array.markers.append(
            create_plane(1, 0.0, 0.0, self.max_z, span, span, thickness, 0.0, 1.0, 0.0))
        
        # maxy
        marker_array.markers.append(
            create_plane(2, 0.0, self.max_y, 0.0, span, thickness, span, 0.0, 0.0, 1.0))

        self.bounds_pub.publish(marker_array)


    def _on_parameter_update(self, params):
        new_min_z = self.min_z
        new_max_z = self.max_z

        for param in params:
            if param.name == 'min_z' and param.type_ == Parameter.Type.DOUBLE:
                new_min_z = float(param.value)
            elif param.name == 'max_z' and param.type_ == Parameter.Type.DOUBLE:
                new_max_z = float(param.value)
            elif param.name in ('cube_max_v', 'tape_h_min', 'tape_h_max', 'tape_s_min',
                                'tape_v_min', 'tape_min_points'):
                setattr(self, param.name, int(param.value))

        if new_min_z > new_max_z:
            return SetParametersResult(
                successful=False,
                reason='min_z must be <= max_z',
            )

        self.min_z = new_min_z
        self.max_z = new_max_z
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePCSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
