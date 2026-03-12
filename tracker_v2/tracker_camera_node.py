"""
ROS 2 node that wraps the OAK-D / DepthAI pipeline for person detection,
BotSORT tracking, target selection, and optional debug visualization.
"""

import os

os.environ.setdefault('QT_LOGGING_RULES', 'qt.qpa.fonts=false')

import pathlib
import time
from contextlib import ExitStack
from dataclasses import dataclass
from enum import Enum

import cv2
import depthai as dai
import numpy as np
import rclpy
# import ultralytics
from ament_index_python.packages import get_package_share_directory
from depthai_nodes.node import ParsingNeuralNetwork
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float32, Float32MultiArray, Int32, String
# from ultralytics.cfg import get_cfg
# from ultralytics.trackers.bot_sort import BOTSORT

NODE_NAME = 'tracker_camera_node'
WINDOW_NAME = 'Tracker V2'
PERSON_CLASS = 0
INFO_PANEL_HEIGHT = 136
CONTROL_MODES = ('IDLE', 'SEARCH', 'TRACK')

COLOR_NORMAL = (0, 200, 0)
COLOR_LOCKED = (0, 100, 255)
COLOR_LOST = (0, 255, 255)
COLOR_PANEL = (28, 28, 28)
COLOR_TEXT = (230, 230, 230)
COLOR_SUBTEXT = (150, 150, 150)


@dataclass
class TrackedPerson:
    track_id: int
    bbox: tuple[int, int, int, int]
    confidence: float

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


class TrackingState(Enum):
    ARMED = 'ARMED'
    LOCKED = 'LOCKED'
    LOST = 'LOST'


class TargetStateMachine:
    def __init__(self, lost_timeout: int):
        self.state = TrackingState.ARMED
        self.target_id: int | None = None
        self.target_bbox: tuple[int, int, int, int] | None = None
        self.lost_frames = 0
        self.lost_timeout = lost_timeout

    def select_target(self, track_id: int, persons: list[TrackedPerson]) -> bool:
        target = next((p for p in persons if p.track_id == track_id), None)
        if target is None:
            return False
        self.state = TrackingState.LOCKED
        self.target_id = track_id
        self.target_bbox = target.bbox
        self.lost_frames = 0
        return True

    def cancel(self) -> None:
        self.state = TrackingState.ARMED
        self.target_id = None
        self.target_bbox = None
        self.lost_frames = 0

    def update(self, persons: list[TrackedPerson]) -> None:
        if self.state == TrackingState.ARMED:
            return

        target = next((p for p in persons if p.track_id == self.target_id), None)
        if target is not None:
            self.state = TrackingState.LOCKED
            self.target_bbox = target.bbox
            self.lost_frames = 0
            return

        self.lost_frames += 1
        if self.target_bbox is not None:
            self.state = TrackingState.LOST

        if self.lost_frames >= self.lost_timeout:
            self.cancel()


class MouseState:
    def __init__(self) -> None:
        self.pending_click: tuple[int, int] | None = None
        self.display_size = (0, 0)
        self.tracking_size = (0, 0)

    def callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and y < self.display_size[1]:
            self.pending_click = (x, y)

    def consume_tracking_click(self) -> tuple[int, int] | None:
        if self.pending_click is None:
            return None
        click_x, click_y = self.pending_click
        self.pending_click = None

        display_w, display_h = self.display_size
        tracking_w, tracking_h = self.tracking_size
        if min(display_w, display_h, tracking_w, tracking_h) <= 0:
            return None

        mapped_x = int(click_x * tracking_w / display_w)
        mapped_y = int(click_y * tracking_h / display_h)
        return mapped_x, mapped_y


class _Boxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32).ravel()
        self.cls = np.asarray(cls, dtype=np.float32).ravel()
        if len(self.xyxy) == 0:
            self.xywh = np.empty((0, 4), dtype=np.float32)
            return
        xc = (self.xyxy[:, 0] + self.xyxy[:, 2]) / 2
        yc = (self.xyxy[:, 1] + self.xyxy[:, 3]) / 2
        bw = self.xyxy[:, 2] - self.xyxy[:, 0]
        bh = self.xyxy[:, 3] - self.xyxy[:, 1]
        self.xywh = np.stack([xc, yc, bw, bh], axis=1)

    def __len__(self):
        return len(self.xyxy)

    def __getitem__(self, idx):
        return _Boxes(self.xyxy[idx], self.conf[idx], self.cls[idx])


def _select_stereo_preset():
    preset_mode = dai.node.StereoDepth.PresetMode
    for name in ('HIGH_DENSITY', 'HIGH_DETAIL', 'DEFAULT'):
        preset = getattr(preset_mode, name, None)
        if preset is not None:
            return preset, name
    raise AttributeError('No supported StereoDepth preset found in depthai build')


def _tracks_to_persons(tracks) -> list[TrackedPerson]:
    persons: list[TrackedPerson] = []
    for track in tracks:
        x1, y1, x2, y2 = (int(track[0]), int(track[1]), int(track[2]), int(track[3]))
        persons.append(
            TrackedPerson(
                track_id=int(track[4]),
                bbox=(x1, y1, x2, y2),
                confidence=float(track[5]),
            )
        )
    return persons


def _detections_to_persons(detections, width: int, height: int) -> list[TrackedPerson]:
    persons: list[TrackedPerson] = []
    for idx, det in enumerate(detections):
        x1 = int(det.xmin * width)
        y1 = int(det.ymin * height)
        x2 = int(det.xmax * width)
        y2 = int(det.ymax * height)
        persons.append(
            TrackedPerson(
                track_id=idx,
                bbox=(x1, y1, x2, y2),
                confidence=float(det.confidence),
            )
        )
    return persons


def _find_nearest_person(px: int, py: int, persons: list[TrackedPerson]) -> TrackedPerson | None:
    for person in persons:
        x1, y1, x2, y2 = person.bbox
        if x1 <= px <= x2 and y1 <= py <= y2:
            return person
    if not persons:
        return None
    return min(persons, key=lambda p: (p.center[0] - px) ** 2 + (p.center[1] - py) ** 2)


def _draw_dashed_rect(
    image: np.ndarray,
    bbox: tuple[int, int, int, int],
    color: tuple[int, int, int],
    thickness: int = 2,
    dash_len: int = 10,
) -> None:
    x1, y1, x2, y2 = bbox
    edges = [
        ((x1, y1), (x2, y1)),
        ((x2, y1), (x2, y2)),
        ((x2, y2), (x1, y2)),
        ((x1, y2), (x1, y1)),
    ]
    for (sx, sy), (ex, ey) in edges:
        dist = int(np.hypot(ex - sx, ey - sy))
        for i in range(0, dist, dash_len * 2):
            t0 = i / max(dist, 1)
            t1 = min((i + dash_len) / max(dist, 1), 1.0)
            p0 = (int(sx + (ex - sx) * t0), int(sy + (ey - sy) * t0))
            p1 = (int(sx + (ex - sx) * t1), int(sy + (ey - sy) * t1))
            cv2.line(image, p0, p1, color, thickness)


def _draw_tracking_frame(
    frame: np.ndarray,
    persons: list[TrackedPerson],
    state_machine: TargetStateMachine,
) -> np.ndarray:
    canvas = frame.copy()
    for person in persons:
        is_target = person.track_id == state_machine.target_id
        color = COLOR_NORMAL
        thickness = 2
        if is_target:
            color = COLOR_LOST if state_machine.state == TrackingState.LOST else COLOR_LOCKED
            thickness = 3

        x1, y1, x2, y2 = person.bbox
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
        label = f'ID {person.track_id} {person.confidence:.0%}'
        cv2.putText(
            canvas,
            label,
            (x1, max(y1 - 8, 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    if state_machine.state == TrackingState.LOST and state_machine.target_bbox is not None:
        _draw_dashed_rect(canvas, state_machine.target_bbox, COLOR_LOST, thickness=2)

    return canvas


def _render_info_panel(
    frame: np.ndarray,
    state_machine: TargetStateMachine,
    fps: float,
    view_mode: str,
    control_mode: str,
    search_radius_m: float,
    search_num_loops: int,
    search_center: tuple[float, float],
    search_ready: bool,
    gps_ready: bool,
) -> np.ndarray:
    panel = np.full((INFO_PANEL_HEIGHT, frame.shape[1], 3), COLOR_PANEL, dtype=np.uint8)

    if state_machine.state == TrackingState.ARMED:
        status_text = 'Click a person to lock on.'
        status_color = (200, 200, 0)
    elif state_machine.state == TrackingState.LOCKED:
        status_text = f'Locked on ID {state_machine.target_id}'
        status_color = (0, 180, 0)
    else:
        status_text = (
            f'Lost target ID {state_machine.target_id} - reacquiring '
            f'({state_machine.lost_frames}/{state_machine.lost_timeout})'
        )
        status_color = (0, 180, 255)

    cv2.putText(
        panel,
        f'State: {state_machine.state.value}',
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        status_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f'Mode: {control_mode}',
        (210, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        COLOR_TEXT,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f'{fps:.1f} fps',
        (frame.shape[1] - 90, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        COLOR_TEXT,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f"View: {'Disparity' if view_mode == 'disparity' else 'Tracking'}",
        (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        COLOR_TEXT,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        status_text,
        (150, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        COLOR_TEXT,
        1,
        cv2.LINE_AA,
    )
    center_x, center_y = search_center
    search_text = (
        f'Search: R={search_radius_m:.1f}m loops={search_num_loops} '
        f'center=({center_x:.1f}, {center_y:.1f}) '
        f'path={"ready" if search_ready else "waiting"} '
        f'gps={"ok" if gps_ready else "wait"}'
    )
    cv2.putText(
        panel,
        search_text,
        (10, 78),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        COLOR_TEXT,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        '[Click] Select  [M] Mode  [R] Recenter  [[/]] Radius  [-/=] Loops',
        (10, 104),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        COLOR_SUBTEXT,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        '[C/Esc] Cancel  [V] Toggle view  [Q] Quit',
        (10, 126),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        COLOR_SUBTEXT,
        1,
        cv2.LINE_AA,
    )

    return np.vstack([frame, panel])


class TrackerCameraNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('fps_limit', 10),
                ('lost_timeout_frames', 30),
                ('target_box_fraction', 0.40),
                ('model_path', ''),
                ('tracker_cfg_path', ''),
                ('enable_gui', True),
                ('show_info_panel', True),
                ('publish_annotated_image', True),
                ('publish_debug_image', True),
                ('publish_disparity_image', True),
                ('enable_disparity', True),
                ('extended_disparity', False),
                ('subpixel', False),
                ('lr_check', True),
                ('image_frame_id', 'camera_link'),
                ('auto_acquire_in_search', True),
                ('auto_return_to_search', True),
                ('ui_search_radius_m', 8.0),
                ('ui_search_radius_step_m', 1.0),
                ('ui_search_num_loops', 3),
            ],
        )

        self.fps_limit = int(self.get_parameter('fps_limit').value)
        self.lost_timeout_frames = int(self.get_parameter('lost_timeout_frames').value)
        self.target_box_fraction = float(self.get_parameter('target_box_fraction').value)
        self.enable_gui = bool(self.get_parameter('enable_gui').value)
        self.show_info_panel = bool(self.get_parameter('show_info_panel').value)
        self.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.publish_disparity_image = bool(self.get_parameter('publish_disparity_image').value)
        self.enable_disparity = bool(self.get_parameter('enable_disparity').value)
        self.extended_disparity = bool(self.get_parameter('extended_disparity').value)
        self.subpixel = bool(self.get_parameter('subpixel').value)
        self.lr_check = bool(self.get_parameter('lr_check').value)
        self.image_frame_id = str(self.get_parameter('image_frame_id').value)
        self.auto_acquire_in_search = bool(self.get_parameter('auto_acquire_in_search').value)
        self.auto_return_to_search = bool(self.get_parameter('auto_return_to_search').value)
        self.ui_search_radius_step_m = float(self.get_parameter('ui_search_radius_step_m').value)

        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.raw_image_pub = self.create_publisher(Image, '/camera/color/image_raw/raw', 10)
        self.debug_image_pub = self.create_publisher(Image, '/tracker/debug_image', 10)
        self.disparity_pub = self.create_publisher(Image, '/tracker/disparity_image', 10)
        self.error_pub = self.create_publisher(Float32MultiArray, '/tracking_error', 10)
        self.state_pub = self.create_publisher(String, '/tracker/state', 10)
        self.target_pub = self.create_publisher(Int32, '/tracker/target_id', 10)
        self.mode_request_pub = self.create_publisher(String, '/tracker/control_mode/request', 10)
        self.auto_track_request_pub = self.create_publisher(Empty, '/tracker/auto_track_request', 10)
        self.auto_search_request_pub = self.create_publisher(Empty, '/tracker/auto_search_request', 10)
        self.search_recenter_pub = self.create_publisher(Empty, '/search/recenter', 10)
        self.search_radius_pub = self.create_publisher(Float32, '/search/set_radius', 10)
        self.search_loops_pub = self.create_publisher(Int32, '/search/set_loops', 10)

        mode_qos = QoSProfile(depth=1)
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        mode_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(Point, '/tracker/select_point', self._select_point_cb, 10)
        self.create_subscription(Empty, '/tracker/cancel', self._cancel_cb, 10)
        self.create_subscription(String, '/tracker/control_mode', self._control_mode_cb, mode_qos)
        self.create_subscription(Float32MultiArray, '/search/status', self._search_status_cb, 10)

        self.state_machine = TargetStateMachine(self.lost_timeout_frames)
        self.mouse_state = MouseState()
        self.view_mode = 'tracking'
        self._pending_select_point: tuple[int, int] | None = None
        self._cancel_requested = False
        self._latest_detections = []
        self._last_disparity_frame: np.ndarray | None = None
        self._prev_time = time.perf_counter()
        self.control_mode = 'SEARCH'
        self.search_radius_m = float(self.get_parameter('ui_search_radius_m').value)
        self.search_num_loops = int(self.get_parameter('ui_search_num_loops').value)
        self.search_center = (0.0, 0.0)
        self.search_ready = False
        self.gps_ready = False

        model_path = self._resolve_model_path(str(self.get_parameter('model_path').value))
        # tracker_cfg_path = self._resolve_tracker_cfg_path(
        #     str(self.get_parameter('tracker_cfg_path').value)
        # )
        # self._tracker = BOTSORT(get_cfg(tracker_cfg_path), frame_rate=self.fps_limit)
        self._exit_stack = ExitStack()
        self._device = dai.Device()
        self.get_logger().info(f'OAK-D platform: {self._device.getPlatformAsString()}')

        (
            self._pipeline,
            self._video_q,
            self._det_q,
            self._disparity_q,
            self._max_disparity,
        ) = self._create_pipeline(model_path)
        # Enter the pipeline context AFTER all nodes and queues are built,
        # matching the `with pipeline: pipeline.start()` pattern from the
        # standalone test.  Entering the context before node creation seals
        # the pipeline graph prematurely in depthai 3.x, causing empty queues.
        self._exit_stack.enter_context(self._pipeline)
        self._pipeline.start()

        self._gui_ready = False
        if self.enable_gui:
            self._setup_gui()

        self.create_timer(1.0 / max(self.fps_limit, 1), self._process)
        self.get_logger().info(
            f'{NODE_NAME} started - fps={self.fps_limit}, '
            f'lost_timeout={self.lost_timeout_frames}, gui={self._gui_ready}'
        )

    def _resolve_model_path(self, configured_path: str) -> str:
        if configured_path:
            path = pathlib.Path(configured_path).expanduser()
        else:
            package_share = pathlib.Path(get_package_share_directory('tracker_v2'))
            path = package_share / 'models' / 'yolo26n.rvc2.tar.xz'

        if not path.exists():
            raise FileNotFoundError(f'Model archive not found: {path}')
        return str(path)

    def _resolve_tracker_cfg_path(self, configured_path: str) -> str:
        if configured_path:
            path = pathlib.Path(configured_path).expanduser()
            if not path.exists():
                raise FileNotFoundError(f'Tracker config not found: {path}')
            return str(path)
        return str(pathlib.Path(ultralytics.__file__).parent / 'cfg/trackers/botsort.yaml')

    def _setup_gui(self) -> None:
        try:
            cv2.namedWindow(WINDOW_NAME)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_state.callback)
            self._gui_ready = True
        except Exception as exc:
            self._gui_ready = False
            self.get_logger().warn(f'GUI disabled: {exc}')

    def _create_pipeline(self, model_path: str):
        pipeline = dai.Pipeline(self._device)

        cam_rgb = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_A
        )
        model_archive = dai.NNArchive(archivePath=model_path)
        nn_node = pipeline.create(ParsingNeuralNetwork).build(
            cam_rgb,
            model_archive,
            fps=self.fps_limit,
        )

        video_q = nn_node.passthrough.createOutputQueue()
        det_q = nn_node.out.createOutputQueue()
        disparity_q = None
        max_disparity = 1.0

        if self.enable_disparity:
            try:
                stereo_preset, stereo_name = _select_stereo_preset()
                mono_left = pipeline.create(dai.node.MonoCamera)
                mono_right = pipeline.create(dai.node.MonoCamera)
                stereo = pipeline.create(dai.node.StereoDepth)

                mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
                mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

                stereo.setDefaultProfilePreset(stereo_preset)
                stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
                stereo.setLeftRightCheck(self.lr_check)
                stereo.setExtendedDisparity(self.extended_disparity)
                stereo.setSubpixel(self.subpixel)

                mono_left.out.link(stereo.left)
                mono_right.out.link(stereo.right)

                max_disparity = max(float(stereo.initialConfig.getMaxDisparity()), 1.0)
                disparity_q = stereo.disparity.createOutputQueue()
                self.get_logger().info(f'Disparity enabled with preset {stereo_name}')
            except Exception as exc:
                self.get_logger().warn(f'Disparity unavailable: {exc}')

        return pipeline, video_q, det_q, disparity_q, max_disparity

    def _select_point_cb(self, msg: Point) -> None:
        self._pending_select_point = (int(msg.x), int(msg.y))

    def _cancel_cb(self, msg: Empty) -> None:
        self._cancel_requested = True

    def _control_mode_cb(self, msg: String) -> None:
        new_mode = str(msg.data).upper().strip() or 'IDLE'
        if new_mode == self.control_mode:
            return
        self.control_mode = new_mode
        if self.control_mode != 'TRACK':
            self.state_machine.cancel()

    def _search_status_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            return
        self.search_radius_m = float(msg.data[0])
        self.search_num_loops = max(1, int(round(msg.data[1])))
        self.search_center = (float(msg.data[2]), float(msg.data[3]))
        self.search_ready = msg.data[4] > 0.5
        self.gps_ready = msg.data[5] > 0.5

    def _request_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.mode_request_pub.publish(msg)

    def _cycle_mode(self) -> None:
        try:
            idx = CONTROL_MODES.index(self.control_mode)
        except ValueError:
            idx = 0
        self._request_mode(CONTROL_MODES[(idx + 1) % len(CONTROL_MODES)])

    def _publish_search_radius(self, radius_m: float) -> None:
        self.search_radius_m = max(1.0, radius_m)
        msg = Float32()
        msg.data = float(self.search_radius_m)
        self.search_radius_pub.publish(msg)

    def _publish_search_loops(self, num_loops: int) -> None:
        self.search_num_loops = max(1, num_loops)
        msg = Int32()
        msg.data = int(self.search_num_loops)
        self.search_loops_pub.publish(msg)

    def _request_recenter(self) -> None:
        self.search_recenter_pub.publish(Empty())

    def _request_auto_track(self) -> None:
        self.auto_track_request_pub.publish(Empty())

    def _request_auto_search(self) -> None:
        self.auto_search_request_pub.publish(Empty())

    def _consume_input_selection(self) -> tuple[int, int] | None:
        point = self._pending_select_point
        if point is not None:
            self._pending_select_point = None
            return point
        return self.mouse_state.consume_tracking_click()

    @staticmethod
    def _choose_auto_target(
        persons: list[TrackedPerson], frame_width: int, frame_height: int
    ) -> TrackedPerson | None:
        if not persons:
            return None
        center_x = frame_width / 2.0
        center_y = frame_height / 2.0
        return min(
            persons,
            key=lambda person: (
                (person.center[0] - center_x) ** 2 + (person.center[1] - center_y) ** 2,
                -person.confidence,
            ),
        )

    def _handle_keypress(self, key: int) -> None:
        if key == ord('q'):
            self.get_logger().info('q pressed - shutting down')
            rclpy.shutdown()
        elif key in (ord('c'), 27):
            self.state_machine.cancel()
        elif key == ord('m'):
            self._cycle_mode()
        elif key == ord('r'):
            self._request_recenter()
        elif key == ord('['):
            self._publish_search_radius(self.search_radius_m - self.ui_search_radius_step_m)
        elif key == ord(']'):
            self._publish_search_radius(self.search_radius_m + self.ui_search_radius_step_m)
        elif key == ord('-'):
            self._publish_search_loops(self.search_num_loops - 1)
        elif key == ord('='):
            self._publish_search_loops(self.search_num_loops + 1)
        elif key == ord('v'):
            self.view_mode = 'disparity' if self.view_mode == 'tracking' else 'tracking'

    def _get_colorized_disparity(self) -> np.ndarray | None:
        if self._disparity_q is None:
            return None
        msg = self._disparity_q.tryGet()
        if msg is None:
            return None
        frame = msg.getFrame()
        normalized = (frame * (255.0 / self._max_disparity)).astype(np.uint8)
        return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

    def _publish_image(self, publisher, frame: np.ndarray) -> None:
        frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.image_frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])

        if frame.ndim == 2:
            msg.encoding = 'mono8'
            msg.step = int(frame.shape[1])
        elif frame.ndim == 3 and frame.shape[2] == 3:
            msg.encoding = 'bgr8'
            msg.step = int(frame.shape[1] * frame.shape[2])
        else:
            raise ValueError(f'Unsupported image shape for ROS Image message: {frame.shape}')

        msg.is_bigendian = 0
        msg.data = frame.tobytes()
        publisher.publish(msg)

    def _publish_state(self) -> None:
        state_msg = String()
        state_msg.data = self.state_machine.state.value
        self.state_pub.publish(state_msg)

        target_msg = Int32()
        target_msg.data = int(self.state_machine.target_id or -1)
        self.target_pub.publish(target_msg)

    def _publish_tracking_error(
        self,
        target: TrackedPerson | None,
        frame_size: tuple[int, int],
    ) -> None:
        height, width = frame_size
        e_lateral = 0.0
        e_distance = 0.0
        confidence = 0.0
        is_tracking = 0.0

        if target is not None and self.state_machine.state == TrackingState.LOCKED:
            x1, y1, x2, y2 = target.bbox
            box_center_x = (x1 + x2) / 2.0
            box_h_frac = (y2 - y1) / float(height)
            e_lateral = (box_center_x - width / 2.0) / (width / 2.0)
            e_distance = self.target_box_fraction - box_h_frac
            confidence = target.confidence
            is_tracking = 1.0

        msg = Float32MultiArray()
        msg.data = [float(e_lateral), float(e_distance), float(confidence), float(is_tracking)]
        self.error_pub.publish(msg)

    def _process(self) -> None:
        if self._cancel_requested:
            self.state_machine.cancel()
            self._cancel_requested = False

        frame_msg = self._video_q.tryGet()
        det_msg = self._det_q.tryGet()

        if det_msg is not None:
            self._latest_detections = [
                detection for detection in det_msg.detections if detection.label == PERSON_CLASS
            ]

        disparity_frame = self._get_colorized_disparity()
        if disparity_frame is not None:
            self._last_disparity_frame = disparity_frame
            if self.publish_disparity_image:
                self._publish_image(self.disparity_pub, disparity_frame)

        if frame_msg is None:
            if self._gui_ready:
                self._handle_keypress(cv2.waitKey(1) & 0xFF)
            return

        frame = frame_msg.getCvFrame()
        height, width = frame.shape[:2]
        self.mouse_state.tracking_size = (width, height)

        now = time.perf_counter()
        fps = 1.0 / max(now - self._prev_time, 1e-6)
        self._prev_time = now

        # boxes = _Boxes(
        #     [
        #         [
        #             det.xmin * width,
        #             det.ymin * height,
        #             det.xmax * width,
        #             det.ymax * height,
        #         ]
        #         for det in self._latest_detections
        #     ],
        #     [det.confidence for det in self._latest_detections],
        #     [PERSON_CLASS] * len(self._latest_detections),
        # )
        # persons = _tracks_to_persons(self._tracker.update(boxes, frame))
        persons = _detections_to_persons(self._latest_detections, width, height)

        selected_point = self._consume_input_selection()
        if selected_point is not None:
            clicked_person = _find_nearest_person(selected_point[0], selected_point[1], persons)
            if clicked_person is not None:
                self.state_machine.select_target(clicked_person.track_id, persons)
                if self.control_mode != 'TRACK':
                    self._request_mode('TRACK')
        elif (
            self.control_mode == 'SEARCH'
            and self.auto_acquire_in_search
            and self.state_machine.state != TrackingState.LOCKED
        ):
            auto_target = self._choose_auto_target(persons, width, height)
            if auto_target is not None:
                self.state_machine.select_target(auto_target.track_id, persons)
                self._request_auto_track()

        prev_tracking_state = self.state_machine.state
        self.state_machine.update(persons)
        if (
            self.control_mode == 'TRACK'
            and self.auto_return_to_search
            and prev_tracking_state != TrackingState.ARMED
            and self.state_machine.state == TrackingState.ARMED
        ):
            self._request_auto_search()
        target = next(
            (person for person in persons if person.track_id == self.state_machine.target_id),
            None,
        )

        self._publish_image(self.raw_image_pub, frame)

        tracking_frame = _draw_tracking_frame(frame, persons, self.state_machine)
        if self.publish_annotated_image:
            self._publish_image(self.image_pub, tracking_frame)

        display = tracking_frame
        if self.view_mode == 'disparity' and self._last_disparity_frame is not None:
            display = self._last_disparity_frame

        if self.show_info_panel:
            debug_frame = _render_info_panel(
                display,
                self.state_machine,
                fps,
                self.view_mode,
                self.control_mode,
                self.search_radius_m,
                self.search_num_loops,
                self.search_center,
                self.search_ready,
                self.gps_ready,
            )
        else:
            debug_frame = display

        if self.publish_debug_image:
            self._publish_image(self.debug_image_pub, debug_frame)

        self._publish_state()
        self._publish_tracking_error(target, (height, width))

        if self._gui_ready:
            self.mouse_state.display_size = (display.shape[1], display.shape[0])
            cv2.imshow(WINDOW_NAME, debug_frame)
            self._handle_keypress(cv2.waitKey(1) & 0xFF)

    def destroy_node(self):
        self._exit_stack.close()
        if self._gui_ready:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackerCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
