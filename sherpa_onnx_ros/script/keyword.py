#!/usr/bin/env python3

import rospy
import nav_msgs.msg
from std_srvs.srv import Empty
from std_msgs.msg import String, Bool
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction
from pathlib import Path
import sounddevice as sd
import sherpa_onnx

class KeywordSpotterNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("keyword_spotter", anonymous=True)

        # Load parameters from ROS parameter server
        self.tokens = rospy.get_param("~tokens", "")
        self.encoder = rospy.get_param("~encoder", "")
        self.decoder = rospy.get_param("~decoder", "")
        self.joiner = rospy.get_param("~joiner", "")
        self.keywords_file = rospy.get_param("~keywords_file", "")
        self.num_threads = rospy.get_param("~num_threads", 1)
        self.provider = rospy.get_param("~provider", "cpu")
        self.max_active_paths = rospy.get_param("~max_active_paths", 4)
        self.keywords_score = rospy.get_param("~keywords_score", 1.0)
        self.keywords_threshold = rospy.get_param("~keywords_threshold", 0.25)
        self.num_trailing_blanks = rospy.get_param("~num_trailing_blanks", 1)
        self.sample_rate = rospy.get_param("~sample_rate", 16000)  # Default to 16kHz
        self.samples_per_read = int(0.1 * self.sample_rate)  # 0.1 second = 100ms

        self.update_params()
        self.current_cmd = None  # Current motion command
        self.control_rate = rospy.Rate(10) # 10Hz control frequency

        # State to control audio input
        self.audio_paused = False  # Default is not paused

        # Check that required files exist
        self.assert_file_exists(self.tokens, "tokens")
        self.assert_file_exists(self.encoder, "encoder")
        self.assert_file_exists(self.decoder, "decoder")
        self.assert_file_exists(self.joiner, "joiner")
        self.assert_file_exists(self.keywords_file, "keywords_file")

        # Initialize the keyword spotter
        self.keyword_spotter = sherpa_onnx.KeywordSpotter(
            tokens=self.tokens,
            encoder=self.encoder,
            decoder=self.decoder,
            joiner=self.joiner,
            num_threads=self.num_threads,
            max_active_paths=self.max_active_paths,
            keywords_file=self.keywords_file,
            keywords_score=self.keywords_score,
            keywords_threshold=self.keywords_threshold,
            num_trailing_blanks=self.num_trailing_blanks,
            provider=self.provider,
        )
        self.stream = self.keyword_spotter.create_stream()

        # Publisher for keyword detection results
        self.result_pub = rospy.Publisher("keyword_result", String, queue_size=10)

        # Publish cmd_vel to control robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.global_path_pub = rospy.Publisher('/move_base/DWAPlannerROS/global_plan', nav_msgs.msg.Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/move_base/DWAPlannerROS/local_plan', nav_msgs.msg.Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

        # Subscriber for controlling audio input
        rospy.Subscriber("audio_playing", Bool, self.audio_playing_callback)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.nav_result_callback)
        rospy.loginfo("Keyword Spotter Node initialized successfully.")

        # Add move_base action clients
        self.move_base_client = SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server(rospy.Duration(5))
        rospy.loginfo("Connected to move_base action server")

    def update_params(self):
        """Load motion command mapping from parameter server"""
        try:
            # Retrieve parameters from private namespace
            self.command_map = rospy.get_param("~command_map")
            self.nav_goals = rospy.get_param("~nav_goals")
        except KeyError:
            # Fallback to default configuration
            rospy.logwarn("Using default command map or nav_goal")
            self.command_map = {
                "前进": (0.3, 0.0),   # (linear.x, angular.z)
                "停止": (0.0, 0.0),
                "左转": (0.3, 0.3),
                "右转": (0.3, -0.3),
                "后退": (-0.3, 0.0)
            }
            self.nav_goals = {
                "去一号目标点": (7.42, -8.23, -0.99, 0.06),
                "去二号目标点": (9.56, -3.34, 0.70, 0.71),
                "去三号目标点": (18.57, -4.71, -0.69, 0.71),
                "回到原点": (0.0, 0.0, -0.71, 0.70),
                "取消导航": (0.0, 0.0, 0.0, 0.0)
            }
        # Parameter type validation
        assert isinstance(self.command_map, dict), "Invalid parameter type"

    def assert_file_exists(self, filepath, param_name):
        """Check if a file exists."""
        if not Path(filepath).is_file():
            rospy.logerr(f"Required file for parameter '{param_name}' does not exist: {filepath}")
            rospy.signal_shutdown(f"Missing required file: {filepath}")

    def audio_playing_callback(self, msg):
        """Callback function to handle audio_playing topic."""
        self.audio_paused = msg.data  # Update the audio paused state
        state = "paused" if self.audio_paused else "resumed"
        rospy.loginfo(f"Audio input {state} based on audio_playing topic.")

    def nav_result_callback(self, msg):
        status_code = msg.status.status
        if status_code == 3:  # succeeded
            rospy.loginfo("Navigation succeeded! Goal reached.")
        elif status_code == 4:  # failed
            rospy.logwarn("Navigation failed! Check obstacles or path.")

    def audio_callback(self, indata, frames, time, status):
        """Callback function for processing audio input."""
        if status:
            rospy.logwarn(f"Audio input error: {status}")

        # If audio is paused, skip processing
        if self.audio_paused:
            return

        # Process the audio input
        audio_data = indata.reshape(-1)  # Flatten the audio data
        self.stream.accept_waveform(self.sample_rate, audio_data)

        # Perform decoding
        while self.keyword_spotter.is_ready(self.stream):
            self.keyword_spotter.decode_stream(self.stream)

        # Get keyword spotting result
        result = self.keyword_spotter.get_result(self.stream)
        if result:
            # Publish the keyword spotting result
            msg = String()
            msg.data = result
            self.result_pub.publish(msg)
            rospy.loginfo(f"Keyword spotted: {result}")

            self.handle_movement_commad(result)

    def handle_movement_commad(self, command):
        """Convert voice command to Twist message"""
       
        # Command parsing logic
        if command in self.command_map:
            linear_x, angular_z = self.command_map[command]
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo(f"Executing motion command: {command}")
        elif command in self.nav_goals:
            if command == "取消导航":
                self._cancel_navigation()
                rospy.loginfo(f"Cancelling navigation!!!")
                return 
            else: 
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map"
                goal.pose.position.x = self.nav_goals[command][0]
                goal.pose.position.y = self.nav_goals[command][1]
                goal.pose.orientation.z = self.nav_goals[command][2]
                goal.pose.orientation.w = self.nav_goals[command][3]
                self.goal_pub.publish(goal)
                rospy.loginfo(f"Publishing nav goal: {command}")
        else:
            rospy.loginfo(f"Publishing nav goal: {command}")

    def _cancel_navigation(self):
        """Core method to terminate navigation"""
        # 1. Cancel active action goals
        try:
            self.move_base_client.cancel_all_goals()
            rospy.logdebug("Action goals cancelled")
        except rospy.ROSException as e:
            rospy.logerr(f"Goal cancel failed: {str(e)}")
        
        # 2. Reset costmap layers (global + local)
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=2)
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmaps()
        except Exception as e:
            rospy.logwarn(f"Costmap clearing failed: {str(e)}")

        # 3. Clean visualization artifacts
        self._clear_global_path()  # Global plan visualization
        self._clear_local_path()   # Local trajectory visualization
        self._clear_goal_markers() # Goal markers
        
        # 4. Execute emergency motion halt
        self._emergency_stop()
        
    def _clear_global_path(self):
        """Clear global path display by publishing empty Path message"""
        empty_path = nav_msgs.msg.Path()
        empty_path.header.stamp = rospy.Time.now()
        self.global_path_pub.publish(empty_path)

    def _clear_local_path(self):
        """Clear local path visualization with empty trajectory"""
        empty_path = nav_msgs.msg.Path()
        empty_path.header.stamp = rospy.Time.now()
        self.local_path_pub.publish(empty_path)

    def _clear_goal_markers(self):
        """Remove all RViz goal markers using DELETEALL action"""
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(del_marker)

    def _emergency_stop(self):
        """Force immediate stop by continuous zero-velocity commands"""
        stop_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now() < stop_time:
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.1)
        
    def continuous_control(self, event):
        """Persistent command sending mechanism"""
        if self.current_cmd:
            self.cmd_vel_pub.publish(self.current_cmd)

    def run(self):
        """Start capturing audio and performing keyword spotting."""
        rospy.loginfo("Starting audio input stream...")
        with sd.InputStream(
            channels=1,
            dtype="float32",
            samplerate=self.sample_rate,
            callback=self.audio_callback,
        ):
            rospy.spin()
            rospy.Timer(rospy.Duration(0.1), self.continuous_control)


if __name__ == "__main__":
    try:
        node = KeywordSpotterNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Keyword Spotter Node.")
