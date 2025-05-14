import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher = self.create_publisher(JointState, '/left_hand_joint_command', 10)
        
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
            msg.name = ['ds10_1200_joint1', 'ds10_1200_joint2', 'ds10_1200_joint3', 
                        'ds10_1200_joint4', 'ds10_1200_joint5', 'ds10_1200_joint6']
            msg.position = interpolated_positions
            msg.effort = [0.5] * 6  # Assuming constant effort for simplicity
            
            # Publish the message
            self.publisher.publish(msg)
            
            time.sleep(0.01)  # Sleep for a short time to simulate periodic updates

    def close_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['index_proximal_joint','middle_proximal_joint', 'pinky_proximal_joint', 'ring_proximal_joint', 'thumb_proximal_yaw_joint','thumb_proximal_pitch_joint']
        msg.position = [1.8, 1.8, 1.8, 1.8, 1.5, 0.8]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher.publish(msg)
        self.get_logger().info('Gripper closed.')

    def open_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['index_proximal_joint','middle_proximal_joint', 'pinky_proximal_joint', 'ring_proximal_joint', 'thumb_proximal_yaw_joint','thumb_proximal_pitch_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0, 1.3, 0.0] # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher.publish(msg)
        self.get_logger().info('Gripper Opened.')
        
    def home_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['index_proximal_joint','middle_proximal_joint', 'pinky_proximal_joint', 'ring_proximal_joint', 'thumb_proximal_yaw_joint','thumb_proximal_pitch_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher.publish(msg)


    

def main(args=None):
    rclpy.init(args=args)
    joint_mover = JointMover()

    home_position = [0.0, 0.0, 1.5, 0.0, -1.5, 0.0] 
    pick_1 = [0.0, -0.5, 1.5, 0.0, 0.0, -1.5]
    pick_2 = [0.265, 0.0, 1.3, 0.0, 0.0, -1.5]
    pick_3 = [0.25, 0.4, 1.37, 0.0, 0.0, -1.5]
    pick_4 = [0.25, 0.43, 1.4, 0.0, -0.3, -1.5]
    
    place_1 = [-1.5, 0.4, 1.35, 0.0, 0.0, -1.5]
    
    pick_5 = [-0.15, 1.1, 0.0, 0.0, 0.8, 0.0]
    
    pick_6 = [-0.15, 1.14, 0.0, 0.0, 0.94, 0.0]
    
    
    # Mustard Bottle

    joint_mover.move_joints(start_positions = home_position, end_positions = pick_1, duration=5)

    joint_mover.open_gripper()
    
    time.sleep(1.0)
    
    joint_mover.move_joints(start_positions = pick_1, end_positions = pick_2, duration=5)
    
    time.sleep(1.0)

    joint_mover.move_joints(start_positions = pick_2, end_positions = pick_3, duration=5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = pick_3, end_positions = pick_4, duration=5)

    time.sleep(1.0)

    joint_mover.close_gripper()

    time.sleep(8.0)

    joint_mover.move_joints(start_positions = pick_4, end_positions = pick_2, duration=7)
    
    time.sleep(2.0)
    
    joint_mover.move_joints(start_positions = pick_2, end_positions = place_1, duration=7)
    
    time.sleep(2.0)
    
    joint_mover.home_gripper()
    
    time.sleep(1.0)
    
    joint_mover.move_joints(start_positions = place_1, end_positions = home_position, duration=5)
    
    # Brick 
    
    joint_mover.home_gripper()

    joint_mover.move_joints(start_positions = home_position, end_positions = pick_5, duration=7)
    
    time.sleep(2.0)
    
    joint_mover.move_joints(start_positions = pick_5, end_positions = pick_6, duration=5)
    
    time.sleep(2.0)
    
    joint_mover.close_gripper()
    
    time.sleep(2.0)
    
    joint_mover.move_joints(start_positions = pick_6, end_positions = pick_5, duration=5)
    
    time.sleep(2.0)
    
    joint_mover.move_joints(start_positions = pick_5, end_positions = place_1, duration=7)
    
    time.sleep(2.0)
    
    joint_mover.open_gripper()
    
    time.sleep(2.0)
    
    joint_mover.move_joints(start_positions = place_1, end_positions = home_position, duration=5)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

