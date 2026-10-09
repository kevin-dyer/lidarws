"""
ROS 2 node: tilt-coordinated 3D capture from a 2D LiDAR + tilt stage.

Subscribes to a ``sensor_msgs/LaserScan``, drives the Arduino tilt
stage through the sweep protocol and publishes the accumulated 3D
point cloud as ``sensor_msgs/PointCloud2``.

The scan/sweep sequencing lives in :mod:`coordinator` and is fully
unit-testable; this node only wires ROS topics/parameters to it.
"""
import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from std_msgs.msg import Bool as BoolMsg
from std_srvs.srv import Trigger

from .cloud_accumulator import (
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    CloudAccumulator,
)
from .coordinator import (
    SweepCoordinator,
    State,
    estimate_max_sweep_samples,
    validate_tilt_states,
)
from .tilt_serial import (
    DEFAULT_STEPS_PER_DEGREE,
    TiltStage,
    TiltStageError,
)
from .topic_names import CAPTURE_COMPLETE_TOPIC, POINT_CLOUD_TOPIC


class LatestFrame:
    """
    Thread-safe holder for the most recent LiDAR frame.

    The coordinator calls ``get()`` (raising when nothing new is
    available); the scan callback calls ``put()``.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None

    def put(self, timestamp, rays):
        with self._lock:
            self._pending = (timestamp, rays)

    def get(self):
        with self._lock:
            if self._pending is None:
                raise TimeoutError('no LiDAR frame ready')
            pending, self._pending = self._pending, None
            return pending


class TiltCoordinatorNode(Node):
    def __init__(self):
        super().__init__('tilt_coordinator')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('tilt_speed_deg_s', 2.0)
        self.declare_parameter('min_range', DEFAULT_MIN_RANGE_M)
        self.declare_parameter('max_range', DEFAULT_MAX_RANGE_M)
        self.declare_parameter('lidar_azimuth_offset_deg', 90.0)
        self.declare_parameter('sample_timeout', 30.0)
        self.declare_parameter('cloud_publish_period', 0.5)
        self.declare_parameter('position_a_target', 0.0)
        self.declare_parameter('position_a_start', -45.0)
        self.declare_parameter('position_a_stop', 45.0)
        self.declare_parameter('capture_on_start', True)

        serial_port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        self._tilt_speed_deg_s = float(
            self.get_parameter('tilt_speed_deg_s').value)
        if not 0.0 < self._tilt_speed_deg_s <= 10.0:
            raise ValueError('tilt_speed_deg_s must be greater than 0 and at most 10')
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        lidar_azimuth_offset = float(
            self.get_parameter('lidar_azimuth_offset_deg').value)
        if not math.isfinite(lidar_azimuth_offset):
            raise ValueError('lidar_azimuth_offset_deg must be finite')
        if min_range < 0.0 or max_range <= min_range:
            raise ValueError(
                'range limits must satisfy 0 <= min_range < max_range')
        capture_on_start = self.get_parameter('capture_on_start').value
        self._cloud_publish_period = float(
            self.get_parameter('cloud_publish_period').value)
        if self._cloud_publish_period < 0.0:
            raise ValueError('cloud_publish_period must be non-negative')
        self._last_cloud_publish_time = 0.0

        self._states = [
            State('A',
                  self.get_parameter('position_a_target').value,
                  self.get_parameter('position_a_start').value,
                  self.get_parameter('position_a_stop').value),
        ]
        for state in self._states:
            state.max_samples = estimate_max_sweep_samples(
                state.sweep_start, state.sweep_stop, self._tilt_speed_deg_s)
        validate_tilt_states(self._states)

        self._cloud = CloudAccumulator(
            min_range, max_range, lidar_azimuth_offset)
        self._latest = LatestFrame()
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self._stop_lidar_client = self.create_client(Trigger, '/stop_motor')
        self._start_sub = self.create_subscription(
            BoolMsg, 'start', self._start_cb, 10)
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pc_pub = self.create_publisher(
            PointCloud2, POINT_CLOUD_TOPIC, cloud_qos)
        self._capture_complete_pub = self.create_publisher(
            BoolMsg, CAPTURE_COMPLETE_TOPIC, cloud_qos)
        self._status_pub = self.create_publisher(String, 'status', 10)

        self.get_logger().info(
            'tilt_coordinator ready (serial=%s baud=%d ranges=[%.2f,%.2f])'
            % (serial_port, baud, min_range, max_range))

        if capture_on_start:
            self._start_capture()
        else:
            self._publish_status('idle (publish true on /tilt_coordinator/start to start)')

    # ------------------------------------------------------------------
    def _publish_status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _scan_cb(self, msg):
        """Buffer the latest LaserScan for the coordination loop."""
        if not self._running:
            return
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        step = float(msg.angle_increment)
        rays = []
        for i in range(len(msg.ranges)):
            az = math.degrees(float(msg.angle_min) + i * step)
            rays.append((az, float(msg.ranges[i])))
        self._latest.put(ts, rays)

    def _start_cb(self, msg):
        if msg.data:
            self._start_capture()
        else:
            self._request_stop()

    # ------------------------------------------------------------------
    def _run_capture(self):
        """Blocking capture loop on a worker thread."""
        state = self._states[0]
        self._capture_complete_pub.publish(BoolMsg(data=False))
        self._publish_status(
            'capture started: one upward sweep from %.1f to %.1f degrees'
            % (state.sweep_start, state.sweep_stop))
        stage = None
        stage_connected = False
        completed_positions = 0
        try:
            stage = TiltStage(serial.Serial(
                self.get_parameter('serial_port').value,
                baudrate=self.get_parameter('baud').value,
                timeout=0.1))
            stage.set_steps_per_degree(DEFAULT_STEPS_PER_DEGREE)
            stage.connect()
            stage_connected = True
            stage.set_speed(self._tilt_speed_deg_s)
            self._publish_status('homing tilt stage before capture')
            stage.home(timeout=self.get_parameter('sample_timeout').value)
            self._publish_status('tilt stage homed; starting capture')
            coordinator = SweepCoordinator(
                stage, self._latest.get, cloud=self._cloud,
                sample_timeout=self.get_parameter('sample_timeout').value,
                clock=time.monotonic,
                on_cloud_update=lambda: self._publish_pointcloud(force=False))
            with self._lock:
                if not self._running:
                    return
            self._publish_status('stage connected, %d point(s) already buffered'
                                 % len(self._cloud))
            for state in self._states:
                with self._lock:
                    if not self._running:
                        break
                self._publish_status('position %s' % state.name)
                reached = coordinator.run_position(state)
                self._publish_status(
                    'position %s done (tilt=%.1f, cloud=%d, stats=%s)' % (
                        state.name, reached, len(self._cloud),
                        dict(self._cloud.stats)))
                self._publish_pointcloud()
                completed_positions += 1
        except TiltStageError as exc:
            self._publish_status('error: %s' % exc)
        except Exception as exc:  # noqa: BLE001 - surface any crash to ROS
            self.get_logger().error('capture crashed: %r' % exc)
            self._publish_status('crashed: %r' % exc)
        finally:
            if stage is not None:
                try:
                    if stage.is_sweeping:
                        stage.stop_sweep(timeout=5.0)
                except TiltStageError as exc:
                    self.get_logger().error(
                        'failed to stop tilt sweep during cleanup: %s' % exc)
                if stage_connected:
                    try:
                        self._publish_status('moving tilt stage to home angle')
                        current_angle = stage.read_angle(timeout=2.0)
                        return_timeout = max(
                            self.get_parameter('sample_timeout').value,
                            abs(current_angle) / self._tilt_speed_deg_s + 10.0,
                        )
                        stage.move_to(0.0, timeout=return_timeout)
                        self._publish_status('tilt stage returned to home angle')
                    except TiltStageError as exc:
                        self.get_logger().error(
                            'failed to move tilt stage to home angle: %s' % exc)
                        self._publish_status(
                            'error moving tilt stage to home angle: %s' % exc)
                stage.close()
            if completed_positions == len(self._states):
                self._capture_complete_pub.publish(BoolMsg(data=True))
                self._stop_lidar_motor()
            else:
                self._publish_status(
                    'capture incomplete; leaving RPLIDAR motor running')
            with self._lock:
                self._running = False
            self._publish_status('capture finished, %d point(s)' % len(self._cloud))

    def _stop_lidar_motor(self):
        """Request the RPLIDAR motor to stop after capture completes."""
        try:
            if not self._stop_lidar_client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError('/stop_motor service was not available')

            completed = threading.Event()
            future = self._stop_lidar_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda result: completed.set() if result.done() else None)
            if not completed.wait(timeout=10.0):
                raise TimeoutError('/stop_motor service call timed out')

            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else 'no response'
                raise RuntimeError('/stop_motor was rejected: %s' % message)
            self._publish_status('RPLIDAR motor stop requested: %s' % response.message)
        except Exception as exc:
            self.get_logger().error('failed to stop RPLIDAR motor: %s' % exc)
            self._publish_status('error stopping RPLIDAR motor: %s' % exc)

    def _publish_pointcloud(self, force=True):
        now = time.monotonic()
        if (not force and
                now - self._last_cloud_publish_time < self._cloud_publish_period):
            return
        self._last_cloud_publish_time = now
        pts = self._cloud.to_numpy()
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tilt_stage'
        msg.height = 1
        msg.width = int(len(pts))
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * max(len(pts), 1)
        msg.is_dense = False
        msg.data = (pts.astype('<f4').tobytes() if len(pts) else b'')
        self._pc_pub.publish(msg)

    def destroy_node(self):
        try:
            with self._lock:
                self._running = False
        finally:
            super().destroy_node()

    # ------------------------------------------------------------------
    def _start_capture(self):
        with self._lock:
            if self._running:
                self._publish_status('capture already running')
                return
            self._running = True
        self._thread = threading.Thread(
            target=self._run_capture, name='capture', daemon=True)
        self._thread.start()

    def _request_stop(self):
        with self._lock:
            self._running = False
        self._publish_status('stop requested')

def main(args=None):
    rclpy.init(args=args)
    node = TiltCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
