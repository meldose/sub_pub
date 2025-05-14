import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import time

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.joint_publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.suction_publisher = self.create_publisher(Bool, '/suction_command', 10)
        self.conveyor_publisher = self.create_publisher(Float64, '/conveyor_topic', 10)
        
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
            self.joint_publisher.publish(msg)
            
            time.sleep(0.01)  # Sleep for a short time to simulate periodic updates

    def suction_command(self, command):
        msg = Bool()
        msg.data = command
        self.suction_publisher.publish(msg)
        if msg.data:
            self.get_logger().info('Gripper Open')
        else:
            self.get_logger().info('Gripper Close')

    def conveyor_command(self, speed):
        flow = Float64()
        flow.data = speed
        self.conveyor_publisher.publish(flow)
        if flow.data > 0:
            self.get_logger().info('Conveyor Moving Forward')
        elif flow.data < 0:
            self.get_logger().info('Conveyor Moving Backward')
        elif flow.data == 0:
            self.get_logger().info('Conveyor Stopped')

def main(args=None):
    rclpy.init(args=args)
    joint_mover = JointMover()


    home_position = [0.0, 0.0, 1.5, 0.0, 1.5, 0.0] 

    # Box 1
    pick_position = [1.0, 0.0, 1.5, 0.0, 1.5, 0.0]
    box1_pick_position = [1.05, 0.75, 0.78, 0.0, 1.57, -0.45]


    box1_place1_position = [-1.0, 0.45, 0.0, 0.0, 2.0, 0.0]
    #box1_place2_position = [-1.35, 0.75, -0.1, 0.0, 2.0, 0.0]
    box1_place2_position = [-1.60, 0.75, -0.1, 0.0, 2.0, 0.0]



    # Box 2
    box2_pick1_position = [-1.5, 0.5, -0.07, 0.1, 2.0, 0.0]
    box2_pick2_position = [-1.5, 0.87, -0.07, 0.1, 2.0, 0.0]

    box2_moving_poisiton = [-1.5, 0.3, -0.07, 0.1, 2.0, 0.0]

    box2_place1_position = [1.5, 0.0, 1.5, 0.0, 1.5, 0.0]
    box2_place2_position = [1.3, 0.75, 0.78, 0.0, 1.57, -0.45]

    # Step_1 (Box 2)

    joint_mover.conveyor_command(1.0)   # Move Forward

    time.sleep(2.0)
    
    joint_mover.move_joints(start_positions = home_position, end_positions = box2_pick1_position, duration = 5)

    time.sleep(1.0)

    joint_mover.conveyor_command(0.0)   # Stopped

    joint_mover.move_joints(start_positions = box2_pick1_position, end_positions = box2_pick2_position, duration = 5)

    time.sleep(3.0)

    joint_mover.suction_command(False) # Close Gripper

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = box2_pick2_position, end_positions = box2_moving_poisiton, duration = 5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = box2_moving_poisiton, end_positions = home_position, duration = 5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = home_position, end_positions = box2_place1_position, duration = 5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = box2_place1_position, end_positions = box2_place2_position, duration = 5)

    joint_mover.suction_command(True) 

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = box2_place2_position, end_positions = box2_place1_position, duration = 5)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = box2_place1_position, end_positions = pick_position, duration = 5)

    


    # ## Palletize Boxes

    # time.sleep(1.0)
    
    # joint_mover.move_joints(start_positions = home_position, end_positions = pick_position, duration = 4)

    # time.sleep(1.0)


    joint_mover.move_joints(start_positions = pick_position, end_positions = box1_pick_position, duration = 4)

    time.sleep(3.0)

    joint_mover.suction_command(False) # Close Gripper

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = box1_pick_position, end_positions = pick_position, duration = 4)

    time.sleep(1.0)

    joint_mover.move_joints(start_positions = pick_position, end_positions = home_position, duration = 4)
    
    joint_mover.move_joints(start_positions = home_position, end_positions = box1_place1_position, duration = 4)
    
    time.sleep(1.0)
    
    joint_mover.move_joints(start_positions = box1_place1_position, end_positions = box1_place2_position, duration = 4)
    
    time.sleep(1.0)
    
    joint_mover.suction_command(True)
    
    time.sleep(1.0)
    
    joint_mover.move_joints(start_positions = box1_place2_position, end_positions = box1_place1_position, duration = 4)

    joint_mover.conveyor_command(-1.0) # Move Backward
    
    time.sleep(1.0)
    
    joint_mover.move_joints(start_positions = box1_place1_position, end_positions = home_position, duration = 4) 


    joint_mover.conveyor_command(0.0)






    rclpy.shutdown()

if __name__ == '__main__':
    main()

