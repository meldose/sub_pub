import rclpy # imported rclpy modules
from rclpy.node import Node # imported Node modules
from sensor_msgs.msg import JointState # imported Joinstate
from geometry_msgs.msg import Twist # imported Twist 
import time

# created class Mipa Rbobot
class MiPA_Robot(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher_motion = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_joint = self.create_publisher(JointState, '/mipa_joint_command', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/mipa_joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.latest_joint_state = None

        self.joint_names = [
            'left_shoulder_x','left_shoulder_y', 'left_shoulder_z', 'left_elbow_y',
            'left_wrist_x','left_wrist_y','left_wrist_z',
            'right_shoulder_x','right_shoulder_y', 'right_shoulder_z', 'right_elbow_y',
            'right_wrist_x','right_wrist_y','right_wrist_z','Torso_lin_z'
        ]

# function for joint state callback
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

# functionf for wait for joint state

    def wait_for_joint_state(self, timeout=5.0):
        start_time = time.time()
        while self.latest_joint_state is None:
            if time.time() - start_time > timeout:
                self.get_logger().warn('Timeout waiting for /mipa_joint_states')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def get_selected_joint_state(self, full_joint_state):
        name_index_map = {name: i for i, name in enumerate(full_joint_state.name)}
        selected_state = JointState()
        selected_state.header.stamp = full_joint_state.header.stamp
        selected_state.name = []
        selected_state.position = []
        selected_state.velocity = []
        selected_state.effort = []

        for name in self.joint_names:
            if name in name_index_map:
                i = name_index_map[name]
                selected_state.name.append(name)
                selected_state.position.append(full_joint_state.position[i] if i < len(full_joint_state.position) else 0.0)
                selected_state.velocity.append(full_joint_state.velocity[i] if i < len(full_joint_state.velocity) else 0.0)
                selected_state.effort.append(full_joint_state.effort[i] if i < len(full_joint_state.effort) else 0.0)

        return selected_state
    
    # function for move joints 

    def move_joints(self, start_positions, end_positions, duration):
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Calculate interpolation factor
            t = (time.time() - start_time) / duration
            # Interpolate between start and end positions
            interpolated_positions = [
                start + (end - start) * t for start, end in zip(start_positions, end_positions)
            ]
            
            # Create and populate the JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['left_shoulder_x','left_shoulder_y', 'left_shoulder_z', 'left_elbow_y','left_wrist_x','left_wrist_y','left_wrist_z',
                        'right_shoulder_x','right_shoulder_y', 'right_shoulder_z', 'right_elbow_y','right_wrist_x','right_wrist_y','right_wrist_z','Torso_lin_z' ]
            msg.position = interpolated_positions
            msg.effort = [0.5] * len(interpolated_positions)  # Ensure effort matches the number of joints
            
            # Publish the message
            self.publisher_joint.publish(msg)

            time.sleep(0.01)  # Sleep for a short time to simulate periodic updates

### function for move robot 
    def move_robot(self, duration, linear_speed, angular_speed):
        twist = Twist()
        rate = 0.1  # 10 Hz
        ramp_duration = 1.0  # seconds
        ramp_steps = int(ramp_duration / rate)

        # Ramp-up phase
        for i in range(ramp_steps):
            scale = (i + 1) / ramp_steps
            twist.linear.x = linear_speed * scale
            twist.angular.z = angular_speed * scale
            self.publisher_motion.publish(twist)
            time.sleep(rate)

        # Steady motion phase
        steady_duration = duration - 2 * ramp_duration
        if steady_duration > 0:
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            start_time = time.time()
            while time.time() - start_time < steady_duration:
                self.publisher_motion.publish(twist)
                time.sleep(rate)

        # Ramp-down phase
        for i in range(ramp_steps):
            scale = 1 - (i + 1) / ramp_steps
            twist.linear.x = linear_speed * scale
            twist.angular.z = angular_speed * scale
            self.publisher_motion.publish(twist)
            time.sleep(rate)

        # Ensure complete stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_motion.publish(twist)

    def left_close_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_index_1_joint', 'left_middle_1_joint', 'left_ring_1_joint', 'left_little_1_joint', 'left_thumb_1_joint','left_thumb_2_joint']
        msg.position = [1.6, 1.6, 1.6, 1.6, 1.2, 0.6]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher_joint.publish(msg)
        self.get_logger().info(' Left Gripper Closed....')

    def right_close_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_index_1_joint', 'right_middle_1_joint', 'right_ring_1_joint', 'right_little_1_joint', 'right_thumb_1_joint', 'right_thumb_2_joint']
        msg.position = [1.6, 1.6, 1.6, 1.6, 1.2, 0.6]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [1.5]
        
        # Publish the message to close the gripper
        self.publisher_joint.publish(msg)
        self.get_logger().info(' Right Gripper Closed.....')

    def left_open_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_index_1_joint', 'left_middle_1_joint', 'left_ring_1_joint', 'left_little_1_joint', 'left_thumb_1_joint','left_thumb_2_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0, 1.2, 0.0]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [1.5]
        
        # Publish the message to close the gripper
        self.publisher_joint.publish(msg)
        self.get_logger().info('Left Gripper Opened....')

    def right_open_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_index_1_joint', 'right_middle_1_joint', 'right_ring_1_joint', 'right_little_1_joint', 'right_thumb_1_joint', 'right_thumb_2_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0, 1.15, 0.0]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher_joint.publish(msg)
        self.get_logger().info('Right Gripper Opened....')

    def left_home_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_index_1_joint', 'left_middle_1_joint', 'left_ring_1_joint', 'left_little_1_joint', 'left_thumb_1_joint','left_thumb_2_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher_joint.publish(msg)
        self.get_logger().info('Left Gripper Home....')

    def right_home_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_index_1_joint', 'right_middle_1_joint', 'right_ring_1_joint', 'right_little_1_joint', 'right_thumb_1_joint', 'right_thumb_2_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher_joint.publish(msg)
        self.get_logger().info('Right Gripper Home....')


# function for main function 

def main(args=None):
    rclpy.init(args=args)
    mipa_robot = MiPA_Robot()

    if not mipa_robot.wait_for_joint_state(timeout=5.0):
        mipa_robot.get_logger().error("Failed to receive joint state. Exiting.")
        rclpy.shutdown()
        return

    # Get filtered joint state
    filtered_state = mipa_robot.get_selected_joint_state(mipa_robot.latest_joint_state)
    start_position = filtered_state.position

    # 'left_shoulder_x','left_shoulder_y', 'left_shoulder_z', 'left_elbow_y','left_wrist_x','left_wrist_y','left_wrist_z',
    # 'right_shoulder_x','right_shoulder_y', 'right_shoulder_z', 'right_elbow_y','right_wrist_x','right_wrist_y','right_wrist_z','Torso_lin_z'
    home_position = [-0.5, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0,
     -0.5, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0, 0.0]

    # right_pick_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #  0.0, -0.3, -0.1, 1.0, 0.0, 0.0, 0.0, 0.2]

    right_pick_1 = [0.0, -0.75, -0.1, 2.0, 0.0, 0.0, 0.0,
    0.0, -0.75, -0.1, 0.85, 0.0, 0.0, 0.0, 0.3]

    right_pick_2 = [0.0, -0.75, -0.1, 2.0, 0.0, 0.0, 0.0,
    0.0, -0.85, -0.1, 0.75, 0.0, 0.0, 0.0, 0.37]

    right_pick_3 = [-0.5, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0,
    0.0, -1.5, -0.1, 1.0, 0.0, 0.0, 0.0, 0.0]

    left_pick_1 = [0.0, -0.75, -0.1, 0.85, 0.0, 0.0, 0.0,
    0.0, -0.75, -0.1, 2.0, 0.0, 0.0, 0.0, 0.3] 
     
    left_pick_2 = [0.0, -0.85, -0.1, 0.75, 0.0, 0.0, 0.0,
    0.0, -0.75, -0.1, 2.0, 0.0, 0.0, 0.0, 0.37]

    left_pick_3 = [0.0, -1.5, -0.1, 1.0, 0.0, 0.0, 0.0,
    0.0, -1.5, -0.1, 1.0, 0.0, 0.0, 0.0, 0.25]


    ### Move Forward ###
    mipa_robot.move_robot(linear_speed=0.5, angular_speed=0.0, duration=2.5)

    time.sleep(2.0)

    
    ### Move Home Position ###
    mipa_robot.move_joints(start_positions=start_position, end_positions=home_position, duration=5)
    
    time.sleep(2.0)


    ### Control MiPA Right Hand ###

    mipa_robot.right_home_gripper()
    
    time.sleep(3.0)
    
    mipa_robot.right_open_gripper()
    
    time.sleep(3.0)

    mipa_robot.move_joints(start_positions=home_position, end_positions=right_pick_1, duration=7)

    time.sleep(2.0)

    mipa_robot.move_joints(start_positions=right_pick_1, end_positions=right_pick_2, duration=5)
    
    time.sleep(7.0) 

    mipa_robot.right_close_gripper()

    time.sleep(7.0)

    mipa_robot.move_joints(start_positions=right_pick_2, end_positions=right_pick_3, duration=5)

    mipa_robot.left_open_gripper()

    time.sleep(2.0)


    ### Control MiPA Left Hand ###


    time.sleep(3.0)

    mipa_robot.move_joints(start_positions=right_pick_3, end_positions=left_pick_1, duration=7)

    time.sleep(2.0)
    
    mipa_robot.move_joints(start_positions=left_pick_1, end_positions=left_pick_2, duration=5)

    time.sleep(7.0) 

    mipa_robot.left_close_gripper()

    time.sleep(7.0) 

    mipa_robot.move_joints(start_positions=left_pick_2, end_positions=left_pick_3, duration=5)

    time.sleep(3.0)


    ### Move Backward

    mipa_robot.move_joints(start_positions=left_pick_3, end_positions=home_position, duration=5)

    time.sleep(2.0)

    mipa_robot.move_robot(linear_speed=-0.5, angular_speed=0.0, duration=2.0)

    time.sleep(2.0)

    ### Turn ###

    mipa_robot.move_robot(angular_speed=1.0, linear_speed=0.0, duration=15.0)

    time.sleep(2.0)

    mipa_robot.move_robot(linear_speed=0.5, angular_speed=0.0, duration=8.0)



    

    rclpy.shutdown()

# calling up main function

if __name__ == '__main__':
    main()
