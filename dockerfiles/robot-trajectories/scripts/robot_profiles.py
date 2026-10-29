"""Per-robot settings, picked with "robot" / --robot. Plain data with no ROS
imports, so both the HTTP API (main.py) and the trajectory scripts load it.
bag_topics: what "record_bag" records, kept to commands and pose estimates."""

ROBOT_PROFILES = {
    # /cmd_vel goes through twist_mux, so e_stop and joystick override still
    # apply. /utlidar/robot_odom is the robot's own odometry; /go2/base/odom and
    # /odometry/filtered are re-stamped/filtered copies of it.
    "go2": {
        "cmd_vel": "/cmd_vel",
        "odom": "/utlidar/robot_odom",
        "bag_topics": ["/cmd_vel", "/utlidar/robot_odom"],
    },
    # Wheel odometry is accurate for distance but slips when turning, so heading
    # comes from the lidar's IMU gyro (mounted upside down: z inverted). The
    # lidar boots in standby and the wheels unpowered, hence the std_srvs/SetBool
    # calls around each run. Speed caps match the robot driver's.
    "roverx": {
        "cmd_vel": "/cmd_vel",
        "odom": "/odom_raw",
        "imu": "/unilidar/imu",
        "imu_z_sign": -1.0,
        "on_start": [("/toggle_lidar", True), ("/toggle_wheels", True)],
        "on_stop": [("/toggle_wheels", False)],
        "max_linear_speed": 0.6,
        "max_angular_speed": 1.0,
        "bag_topics": ["/cmd_vel", "/odom_raw", "/odom", "/unilidar/imu"],
    },
}

NO_PROFILE = {"cmd_vel": "/cmd_vel", "odom": ""}
