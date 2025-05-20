import rclpy # imported rclpy modules
from rclpy.node import Node # imported Node modules
from sensor_msgs.msg import JointState # imported Joinstate
import time # imported time module


# created clss JointMover
class JointMover(Node):
    
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        
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

# created function for close gripper
    def close_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['finger_joint']
        msg.position = [0.8]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher.publish(msg)
        self.get_logger().info('Gripper closed.')

# created function for open gripper
    def open_gripper(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['finger_joint']
        msg.position = [0.0]  # Set the position to close the gripper (adjust according to your gripper's range)
        msg.effort = [0.5]
        
        # Publish the message to close the gripper
        self.publisher.publish(msg)
        self.get_logger().info('Gripper Opened.')

    
# created the main function 
def main(args=None):
    rclpy.init(args=args)
    joint_mover = JointMover()

    


    home_position = [0.0, 0.0, 1.5, 0.0, 1.5, 0.0] 

    #Nvidia Cube
    pick_position = [0.0, 0.942478, 1.0472, 0.0, 1.22173, -1.5708] 
    place_position = [-1.22173, 0.383972, 1.13446, 0.0, 1.22173, -1.5708] 
    pick2_position = [0.523599, 0.872665, 1.13446, 0.0, 1.22173, -1.0472] 
    place2_position = [-1.22173, 0.383972, 1.13446, 0.0, 1.22173, -1.5708]
    pick3_position = [-1.22173, 0.45, 1.13446, 0.0, 1.65, -1.5708]


    ## Nvidia Pick & Place
    
    joint_mover.open_gripper()
    
    joint_mover.move_joints(start_positions = home_position, end_positions = pick2_position, duration=5)

    time.sleep(1.0)

    joint_mover.close_gripper()

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = pick2_position, end_positions = place2_position, duration=5)
    
    time.sleep(1.0)

    joint_mover.move_joints(start_positions = place2_position, end_positions = pick3_position, duration=5)

    time.sleep(1.0)

    joint_mover.open_gripper()

    joint_mover.move_joints(start_positions = pick3_position, end_positions = place2_position, duration=5)

    time.sleep(1.0)

    # joint_mover.move_joints(start_positions = place2_position, end_positions = pick3_position, duration=5)

    # joint_mover.close_gripper()

    # time.sleep(1.0)

    # joint_mover.move_joints(start_positions=pick3_position, end_positions = home_position, duration=5) 



    ## Mustard Pick & Place
    mustard_pick1_position = [-1.55, -0.03, 1.63, 0.0, 0.73, -1.57]
    mustard_pick2_position = [-1.59, 0.02, 1.94, -0.03, -0.22, -1.57]
    mustard_place1_position = [1.55, 0.00, 1.63, 0.0, 0.52, -1.57]
    mustard_place2_position = [1.95, 0.7, 1.17, 0.00, -0.14, -1.57]

    joint_mover.open_gripper()

    joint_mover.move_joints(start_positions = place2_position, end_positions = mustard_pick1_position, duration=5)


    # joint_mover.move_joints(start_positions = home_position, end_positions = mustard_pick1_position, duration=5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = mustard_pick1_position, end_positions = mustard_pick2_position, duration=5)

    time.sleep(1.0)

    joint_mover.close_gripper()

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = mustard_pick2_position, end_positions = mustard_pick1_position, duration=5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = mustard_pick1_position, end_positions = mustard_place1_position, duration=7)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = mustard_place1_position, end_positions = mustard_place2_position, duration=8)

    time.sleep(1.0)

    joint_mover.open_gripper()

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = mustard_place2_position, end_positions = mustard_place1_position, duration=8)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = mustard_place1_position, end_positions = home_position, duration=5)
    
    rclpy.shutdown()


# calling the main function
if __name__ == '__main__':
    main()

