"""
Semantic object detector node.

Consumes synchronized RGB + depth frames, runs YOLO detection, back-projects each
detection to a 3D point, and republishes the result in the navigation frame as a
SemanticObjectArray for the semantic costmap layer to consume.

Design contract with the costmap layer:
    This node is STATELESS. It reports only "what I see right now, in this frame".
    It holds no memory across frames: no tracking, no TTL, no decay. All temporal
    reasoning (object persistence, confidence decay, association) lives in the
    costmap layer. Keeping time-dependent logic in exactly one place avoids two
    components disagreeing about when an object stopped existing.

    Consequence: an array is published on EVERY processed frame, including an empty
    one when nothing is detected. The empty array is a positive signal meaning
    "camera is alive and currently sees no objects" - the layer needs it to decay
    stale objects. Silence would be ambiguous (no objects? or detector crashed?).
"""

import cv2
import message_filters
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from ultralytics import YOLO
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

# TF libraries
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# Custom messages
from semantic_nav_interfaces.msg import SemanticObject, SemanticObjectArray


class SemanticDetectorNode(Node):
    def __init__(self):
        super().__init__('semantic_detector_node')

        # --- Parameters -----------------------------------------------------
        # Topic names are parameters, not constants: the TB4 sim exposes the OAK-D
        # under /oakd/rgb/preview/*, but a different robot or a real OAK-D will not.
        self.declare_parameter('rgb_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('sem_pub_topic', '/semantic_objects')

        self.declare_parameter('model_weights', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.4)

        # Class whitelist. Deliberately NOT all 80 COCO classes.
        # A costmap cell means "do not drive here", so the criterion for inclusion is
        # "does this class change navigation behaviour?", not "can YOLO recognize it?".
        #   - Non-obstacles (cell phone, book) would create phantom keep-out zones.
        #   - Static furniture (chair, table) is already seen by the lidar layer;
        #     duplicating it here adds cost without adding information.
        #   - Every extra class is another chance for a false positive to lock the robot.
        # What the camera adds over the lidar is CLASS-CONDITIONAL behaviour: a lidar
        # sees a person as an anonymous cylinder, but knowing it is a person justifies
        # a larger social-distance radius because people move and collisions cost more.
        self.declare_parameter('class_list', ['person', 'dog', 'cat'])
        self.declare_parameter('class_radii', [2.0, 1.0, 1.0])

        self.declare_parameter('target_frame', 'odom')

        # Depth gating. Values are in METRES because the sim publishes 32FC1.
        # (A 16UC1 camera would publish millimetres and every threshold here would
        # be off by 1000x - verify the encoding before trusting these numbers.)
        self.declare_parameter('max_depth', 10.0)      # metres  -> float
        self.declare_parameter('min_valid_pixels', 10)  # pixel count -> int

        # Debug visualization parameters
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/semantic_debug_image')

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.cam_info_topic = self.get_parameter('camera_info_topic').value
        self.sem_pub_topic = self.get_parameter('sem_pub_topic').value
        self.model_weights = self.get_parameter('model_weights').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.class_list = self.get_parameter('class_list').value
        self.class_radii = self.get_parameter('class_radii').value
        self.target_frame = self.get_parameter('target_frame').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_valid_pixels = self.get_parameter('min_valid_pixels').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.bridge = CvBridge()
        self.model = YOLO(model=self.model_weights)

        # Single source of truth: membership in this dict IS the whitelist, and the
        # value is the keep-out radius. One lookup answers "should I keep this?" and
        # "how big is it?" - no chance of the two lists drifting apart.
        self.radius_map = dict(zip(self.class_list, self.class_radii))

        # --- Camera intrinsics ----------------------------------------------
        # Latched once: intrinsics are fixed for the life of the camera.
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        # --- TF --------------------------------------------------------------
        # node=self is REQUIRED, not cosmetic. Without it the Buffer builds its own
        # system clock and ignores use_sim_time, so the node lives in sim time while
        # its TF buffer lives in wall time. The symptom is a permanent, nonsensical
        # "extrapolation into the future" error - the two clocks are simply in
        # different eras.
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscriptions ----------------------------------------------------
        # Sensor QoS (best-effort) is mandatory here: the sim publishes camera topics
        # as best-effort, and a reliable subscriber will never match a best-effort
        # publisher. The failure is silent - no error, just no messages ever.
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.cam_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)

        # RGB and depth are published independently and their stamps never match
        # exactly, so pair them approximately rather than requiring exact equality.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.05   # 50 ms pairing tolerance
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Reliable QoS on the output: this is a derived message consumed by the C++
        # costmap layer, not raw sensor data. The layer's subscription must match.
        self.object_pub = self.create_publisher(
            SemanticObjectArray,
            self.sem_pub_topic,
            10
        )
        
        # Debug image publisher
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info('Semantic detector node started, awaiting TF and camera data...')

    def camera_info_callback(self, msg):
        """Latch the pinhole intrinsics from the first CameraInfo message."""
        if not self.camera_info_received:
            # K is row-major [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info("Camera parameters obtained.")

    def synchronized_callback(self, rgb_msg, depth_msg):
        """
        Process one synchronized RGB+depth pair into a SemanticObjectArray.

        Control-flow rule: `return` aborts the whole FRAME and is reserved for
        failures that make the frame unusable (no intrinsics, decode failure, no TF).
        `continue` skips a single DETECTION. One bad depth patch must never discard
        the other objects in the frame - and must never suppress the publish, since
        an unpublished frame is indistinguishable from a dead detector.
        """
        if not self.camera_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Image transformation error: {str(e)}")
            return
        
        # Take the copy of cv image for debugging
        debug_img = cv_image.copy() if self.publish_debug_image else None

        # Clip against the DEPTH image, since that is the array being indexed.
        # (Here RGB and depth are both 320x240; if they ever differ, the pixel
        # correspondence below breaks and needs an explicit rescale.)
        h_img, w_img = cv_depth.shape[:2]

        # --- TF lookup: once per frame, not once per object --------------------
        # All detections share one image stamp, so they share one transform.
        #
        # KNOWN TRADE-OFF - latest-available transform instead of exact-stamp:
        # The sim runs at RTF ~0.44 (4GB VRAM + heavy warehouse world), and image
        # stamps arrive ~1s AHEAD of the newest available TF. An exact-stamp lookup
        # therefore extrapolates into the future on every single frame and fails.
        # Time() means "give me the newest transform you have".
        #
        # The cost is a pose error while moving: the TF may be up to ~1s stale, so at
        # 0.3 m/s an object lands ~30 cm off, and worse during rotation. This is
        # tolerable here because it is well inside the 2.0 m person radius, and the
        # layer's decay refreshes objects continuously. On real hardware, or at
        # RTF ~= 1, revert to rgb_msg.header.stamp - the accuracy is free there.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                rgb_msg.header.frame_id,
                Time(),               # zero time == latest available
                Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF Transform error: {ex}")
            return

        arr_msg = SemanticObjectArray()
        # Stamp with the IMAGE time, not now(): the layer's TTL must reason about when
        # the observation was made, not when this node happened to finish processing.
        arr_msg.header.stamp = rgb_msg.header.stamp
        # Positions below are expressed in target_frame, so the header must say so -
        # declaring the optical frame here would make the layer place objects wrongly.
        arr_msg.header.frame_id = self.target_frame

        result = self.model(cv_image, conf=self.conf_threshold, verbose=False)[0]
        boxes = result.boxes
 
        for box in boxes:
            b = box.cpu().numpy()
            cls_idx = int(b.cls)
            name = self.model.names[cls_idx]   # .cls is an index; names maps it to a string
            conf = float(b.conf)
 
            # astype(int) on the whole array - element-wise int() assignment would be
            # silently cast back to float by numpy and break the slicing below.
            x1, y1, x2, y2 = b.xyxy[0].astype(int)
 
            # Whitelist check first: never do depth work for an object we will discard.
            if name not in self.radius_map:
                if self.publish_debug_image:
                    # Outline, not filled: a filled box would hide the very object we
                    # are trying to look at. thickness=-1 only makes sense for shapes
                    # whose interior we do not care about.
                    cv2.rectangle(
                        img=debug_img, pt1=(x1, y1), pt2=(x2, y2),
                        color=(128, 128, 128), thickness=2
                    )
                    # Label sits at the box, not at a fixed screen position - with a
                    # hardcoded org every label lands on top of every other one.
                    # y is clamped so a box touching the top edge still gets a visible
                    # label instead of drawing off-screen at a negative y.
                    cv2.putText(
                        img=debug_img, text=f"{name} (not in whitelist)",
                        org=(x1, max(y1 - 5, 12)),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4,
                        color=(128, 128, 128), thickness=1
                    )
                continue
 
            w_bbox, h_bbox = (x2 - x1), (y2 - y1)
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2
 
            # --- Robust depth -------------------------------------------------
            # Sampling the single centre pixel is fragile: on a person it can land
            # between the legs and read the wall behind them, or land on a NaN.
            # Instead take the median over the inner 50% of the bbox - the median is
            # insensitive to the background pixels that leak in at the edges.
            #
            # Patch SIZE comes from the bbox; clip bounds come from the image. Mixing
            # these up is subtle and silent: numpy reads a negative index from the far
            # side of the array instead of raising.
            u_min = max(0, u - w_bbox // 4)
            u_max = min(w_img, u + w_bbox // 4)
            v_min = max(0, v - h_bbox // 4)
            v_max = min(h_img, v + h_bbox // 4)
 
            patch = cv_depth[v_min:v_max, u_min:u_max]   # [row, col] == [v, u]
 
            # Mask BEFORE taking the median - a single NaN poisons np.median, and
            # out-of-range returns (large finite values) would drag it off the object.
            valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < self.max_depth)]
 
            # Guard before the median: an empty array yields NaN plus a warning, and a
            # NaN position would propagate all the way into the costmap unnoticed.
            if valid.size < self.min_valid_pixels:
                if self.publish_debug_image:
                    cv2.rectangle(
                        img=debug_img, pt1=(x1, y1), pt2=(x2, y2),
                        color=(0, 255, 255), thickness=2
                    )
                    # The pixel count is the whole point of this label: it is the only
                    # way to tell whether min_valid_pixels is set sensibly or is quietly
                    # throwing away good detections.
                    cv2.putText(
                        img=debug_img, text=f"{name} REJECTED: depth ({valid.size} px)",
                        org=(x1, max(y1 - 5, 12)),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4,
                        color=(0, 255, 255), thickness=1
                    )
                continue
 
            z = float(np.median(valid))
 
            # --- Back-projection (pinhole) --------------------------------------
            # Inverting u = fx*(X/Z) + cx: subtract the principal point to get the
            # offset from the optical axis, divide by focal length to get a normalized
            # ray direction, multiply by depth to travel along that ray.
            # Result is in the OPTICAL convention (X right, Y down, Z forward), which
            # is why frame_id must be *_optical_frame - tf2 handles the rest.
            x_cam = (u - self.cx) * z / self.fx
            y_cam = (v - self.cy) * z / self.fy
            z_cam = z
 
            pt_in_camera = PointStamped()
            pt_in_camera.header.frame_id = rgb_msg.header.frame_id
            pt_in_camera.header.stamp = rgb_msg.header.stamp
            pt_in_camera.point.x = float(x_cam)
            pt_in_camera.point.y = float(y_cam)
            pt_in_camera.point.z = float(z_cam)
 
            pt_in_target = tf2_geometry_msgs.do_transform_point(pt_in_camera, transform)
 
            obj = SemanticObject()
            obj.class_id = name
            obj.position = pt_in_target.point
            obj.confidence = conf          # instantaneous; the layer decays it over time
            obj.track_id = -1              # -1 == "no identity", stateless mode.
                                           # Not 0, which would read as a valid track.
                                           # Switching to model.track(persist=True) later
                                           # lets the layer key its TTL per track instead
                                           # of re-associating by proximity.
            obj.cost_radius = float(self.radius_map[name])
            arr_msg.objects.append(obj)
 
            # Accepted detection. This block must live inside the loop: x1, u_min, name
            # and friends are per-detection. Drawing after the loop would render only the
            # last box - and crash outright when there were no detections at all, since
            # the names would never have been bound.
            if self.publish_debug_image:
                cv2.rectangle(
                    img=debug_img, pt1=(x1, y1), pt2=(x2, y2),
                    color=(0, 255, 0), thickness=2
                )
                # The patch outline is the single most useful thing on this image: it
                # shows exactly which pixels the median depth was taken from. If it is
                # sitting on the wall behind the person rather than on their body, the
                # depth is wrong and the object will land in the wrong costmap cell.
                cv2.rectangle(
                    img=debug_img, pt1=(u_min, v_min), pt2=(u_max, v_max),
                    color=(255, 0, 0), thickness=1
                )
                cv2.putText(
                    img=debug_img, text=f"{name} {conf:.2f} | {z:.2f}m",
                    org=(x1, max(y1 - 5, 12)),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4,
                    color=(0, 255, 0), thickness=1
                )
                # The exact pixel fed into the back-projection. If this dot is not on the
                # person, neither is the 3D point.
                cv2.circle(
                    img=debug_img, center=(int(u), int(v)), radius=2,
                    color=(0, 0, 255), thickness=-1
                )
 
        # Unconditional, outside the loop. An empty objects[] is the "nothing here
        # right now" signal the layer relies on to decay stale objects.
        self.object_pub.publish(arr_msg)
 
        # Also unconditional: an unannotated frame still tells you the camera is alive.
        if self.publish_debug_image:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = rgb_msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
