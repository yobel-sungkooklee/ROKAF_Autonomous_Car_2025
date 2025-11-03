# ROKAF Autonomous Car

Python ROS stack for the ROKAF autonomous car training program. The project
combines classical lane following, visual mission triggers, YOLO-based object
detection, and peripheral feedback (I2C LCD) to execute scripted missions on a
small ground vehicle.

## Key Capabilities
- **Lane following** – bird’s-eye-view projection, Hough-based lane detection,
  and curvature/lateral error compensation published on `/cmd_vel`.
- **Green mission trigger tracking** – SORT-based multi-object tracking of green
  banners on `/main_camera/image_raw/compressed`, providing a mission index for
  choreography.
- **YOLO object detection** – real-time inference of `alli`, `alli_tank`,
  `enem`, and `enem_tank` classes via Ultralytics YOLO with results on
  `/detected_bboxes`.
- **Mission orchestration** – `af_main.py` coordinates lane following, scripted
  turns, target tracking, and LCD messaging.
- **Support utilities** – ArUco marker identification/pose nodes and an I2C LCD
  subscriber for field debugging.

## Repository Layout
```
af_main.py                     # Mission controller orchestrating modes
lane_follower.py               # Lane detection & control (classical pipeline)
green_object_tracking.py       # Green banner tracker using SORT
green_detect_macro.py          # Simpler green detector with dwell timer
infer_ros.py                   # Ultralytics YOLO ROS bridge
lcd_subscriber.py              # Status display on 16x2 I2C LCD
I2C_LCD_driver.py              # LCD hardware driver
aruco_marker_id_printer.py     # ArUco ID monitor
aruco_marker_pose_detector.py  # ArUco pose estimation & visualization
bagfiles/airforce_{train,test}.bag  # Sample ROS bags
best.pt                        # YOLO weights (alli/enem classes)
sort/                          # Expected to hold SORT tracker implementation
```

## ROS Nodes & Topics
| Node | Script | Subscribes | Publishes | Purpose |
| --- | --- | --- | --- | --- |
| Lane detection | `lane_follower.py` | `/usb_cam/image_raw/compressed` | `/cmd_vel` | Provides baseline steering using BEV + Hough pipeline. |
| Green tracker | `green_object_tracking.py` | `/main_camera/image_raw/compressed` | (none) | Tracks green markers, increments `ColorFilterTracker.index`. |
| Green macro | `green_detect_macro.py` | `/main_camera/image_raw/compressed` | (none) | Standalone green detector alternative (maintains its own node). |
| YOLO detector | `infer_ros.py` | `/main_camera/image_raw/compressed` | `/detected_bboxes` | Runs Ultralytics YOLO using `best.pt`. |
| Mission orchestrator | `af_main.py` | `/detected_bboxes` | `/cmd_vel`, `/lcd_str_1` | Switches between lane following, target acquisition, and scripted turns. |
| LCD subscriber | `lcd_subscriber.py` | `/lcd_str_1` | I2C LCD | Renders mission status lines. |
| ArUco ID printer | `aruco_marker_id_printer.py` | `/usb_cam/image_raw/compressed` | (logs/UI) | Displays detected ArUco IDs. |
| ArUco pose detector | `aruco_marker_pose_detector.py` | `/usb_cam/image_raw/compressed`, `/usb_cam/camera_info` | (logs/UI) | Estimates pose/yaw for calibration. |

> **Note:** `green_object_tracking.py` expects a `sort/sort.py` module. Add the
> SORT implementation (e.g., from the original repo) to avoid import errors.

## Dependencies
- ROS 1 (`rospy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `detection_msgs`)
- OpenCV (`opencv-python`), NumPy
- `cv_bridge` (Ubuntu/ROS package)
- Ultralytics `YOLO` (pip package `ultralytics`)
- `smbus` or `smbus2` for the LCD driver
- Python 3.8+ recommended (matches `.pyc` artifacts in `__pycache__`)

Install Python dependencies (example):
```bash
pip install numpy opencv-python ultralytics smbus2
# cv_bridge and ROS messages are installed via apt in ROS environments
```

## Running the Stack
1. Launch the ROS master:
   ```bash
   roscore
   ```
2. Start camera drivers or play back a bag file, e.g.:
   ```bash
   rosbag play bagfiles/airforce_train.bag --loop
   ```
3. Launch perception nodes (example workflow used on the robot):
   ```bash
   roslaunch mm_cam usb_cam.launch
   roslaunch omo_r1mini_bringup omo_r1mini_bringup.launch
   python lane_follower-PID.py
   rosrun ROKAF_Autonomous_Car green_object_tracking.py
   rosrun ROKAF_Autonomous_Car infer_ros.py
   ```
   - `lane_follower-PID.py` opens several OpenCV visualization windows
     (`Original`, `BEV`, `Color filter`, `thresh`, `Sliding Window`) to show the
     processing stages in real time.
   - Ensure `sort/sort.py` is present before running the green tracker.
4. Run the mission controller:
   ```bash
   rosrun ROKAF_Autonomous_Car af_main.py
   ```
5. (Optional) Run support nodes as needed: `lcd_subscriber.py`,
   `aruco_marker_id_printer.py`, `aruco_marker_pose_detector.py`.

The orchestrator assumes `lane_detect()` is instantiated in-process so that it
can publish the Twist stored in `lf.speed`. Avoid running `lane_follower.py`
standalone *and* importing it elsewhere simultaneously, since it calls
`rospy.init_node` within the class constructor.

## Data & Models
- `bagfiles/` contains recorded ROS sessions for testing perception and control.
- `best.pt` is a trained Ultralytics YOLO model tailored to the four mission
  classes; keep it alongside `infer_ros.py` or adjust the model path.

## Development Notes
- When vendoring SORT, place `sort.py` (and any tracker dependencies) under the
  `sort/` directory so `from sort.sort import Sort` resolves correctly.
- If multiple nodes need the lane follower output, consider refactoring
  `lane_detect` to publish its Twist while returning a reference for reuse.
- Create ROS launch files to coordinate the node graph for repeatable demos.

Happy driving!
