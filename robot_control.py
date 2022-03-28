import urx
import time
from yaml import Loader 
from yaml import load as load_yaml
import transforms3d
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation
import logging
import sys

class URController:
    '''
    Class for controll the UR robot
    '''
    def __init__(self, config_path):
        # Open configuration file
        with open(config_path, 'r') as f:
            configs = load_yaml(f, Loader=Loader)

        # Get variables from configuration file
        self.IP = configs['IP']
        self.photo_pose = configs['photo_jpose']
        self.place_pose = configs['place_jpose']
        # self.end_pose = configs['end_jpose']

        self.camera_to_tcp_transform = configs['camera_to_tcp_transform']
        self.tcp_to_gripper_transform = configs['tcp_to_gripper_transform']
        
        self.camera_above_pose_height = configs['camera_above_pose_height']
        self.gripper_above_pose_height = configs['gripper_above_pose_height']
        self.box_height = configs['box_height']
        self.object_height = configs['object_height']

        self.photo_pose_acc = configs['photo_pose_acc']
        self.photo_pose_acc = configs['photo_pose_acc']
        self.place_pose_acc = configs['place_pose_acc']
        self.place_pose_vel = configs['place_pose_vel']
        self.above_cam_acc = configs['above_cam_acc']
        self.above_cam_vel = configs['above_cam_vel']
        self.above_gripper_acc = configs['above_gripper_acc']
        self.above_gripper_vel = configs['above_gripper_vel']
        self.pick_down_acc = configs['pick_down_acc']
        self.pick_down_vel = configs['pick_down_vel']
        self.pick_up_acc = configs['pick_up_acc']
        self.pick_up_vel = configs['pick_up_vel']

        # Check given pose heights
        self.log = logging.getLogger(__name__)
        if self.camera_above_pose_height <= (self.camera_to_tcp_transform[2]+self.tcp_to_gripper_transform[2]+self.object_height):
            self.log.error("Camera above pose height is smaller than the z distance between camera and gripper")
            sys.exit(0)

        if self.gripper_above_pose_height <= self.object_height:
            self.log.error("Gripper above pose height is smaller than the height of object")
            sys.exit(0)

        self.rob = urx.Robot(self.IP)
        time.sleep(1)
        # Compose the transformation matrix between the camera and tcp
        euler_angles = self.camera_to_tcp_transform[3:]
        self.camera_to_tcp_transform = transforms3d.affines.compose(self.camera_to_tcp_transform[0:3], transforms3d.euler.euler2mat(*euler_angles, axes='sxyz'), [1, 1, 1])
        # Compose the transformation matrix between the tcp and gripper
        euler_angles = self.tcp_to_gripper_transform[3:]
        self.tcp_to_gripper_transform = transforms3d.affines.compose(self.tcp_to_gripper_transform[0:3], transforms3d.euler.euler2mat(*euler_angles, axes='sxyz'), [1, 1, 1])

    
    def set_loglevel(self, level=logging.WARNING):
        '''
        Set loglevel of robot controller class

        arguments:
         - level (enum (logging.levels)): Treshold level for logging (if smaller than WARNING will be set to DEBUG automatically)
        '''
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(message)s')
        handler.setFormatter(formatter)
        self.log.addHandler(handler)
        if level >= logging.WARNING:
            self.log.setLevel(logging.WARNING)
        else:
            self.log.setLevel(logging.DEBUG)

    def get_camera_height(self):
        '''
        Get the height of the camera
        '''
        camera_z = np.array(self.rob.getl())[2]*1000 + self.camera_to_tcp_transform[2][3]
        return camera_z

    def move_to_photo_pose(self):
        '''
        Moves the robot to photo pose
        '''
        self.log.info("--------MOVING TO PHOTO POSE--------")
        self.rob.movej(self.photo_pose, self.photo_pose_acc, self.photo_pose_acc, wait=True)
        
    def move_to_place_pose(self):
        '''
        Moves the robot to place pose
        '''
        self.log.info("--------MOVING TO PLACE POSE--------")
        self.rob.movej(self.place_pose, self.place_pose_acc, self.place_pose_vel, wait=True)

    def calc_above_point_for_camera(self, point):
        '''
        Calculates the robot tcp x,y,z,ry,ry,rz position and orientation where the camera is above the object

        arguments:
            - point (list): object x,y,z position in camera frame
        '''
        point.append(1)
        point[2] -= self.camera_above_pose_height  # photo pose height in mm, when camera is directly above the object
        self.log.info("point: ", point)
        # Get the translation between the camera frame and tcp
        camera_to_tcp_translation = list(transforms3d.affines.decompose44(self.camera_to_tcp_transform)[0])
        camera_to_tcp_translation.append(0)
        # Get the tcp target in camera frame
        tcp_target_in_camera_frame = np.array(camera_to_tcp_translation) + np.array(point)
        self.log.info("transformed point: ", tcp_target_in_camera_frame)
        # Calculate the target point in tcp frame
        point_in_tcp = np.dot(np.linalg.inv(np.array(self.camera_to_tcp_transform)), np.array(tcp_target_in_camera_frame))
        self.log.info("point_in_tcp: ", point_in_tcp)
        # Get actual position of tcp
        tcp_actual_pose = self.rob.getl()   # get in meter
        # Calculate robot to tcp transformation
        robot_to_tcp_transform = transforms3d.affines.compose(np.array(tcp_actual_pose[0:3])*1000, Rotation.from_rotvec(np.array(tcp_actual_pose[3:])).as_matrix(), [1, 1, 1])
        # Calculate the position for the tcp in robot frame, where the camera is above the object
        point_above_in_robot = np.dot(robot_to_tcp_transform, point_in_tcp)
        pose = np.append(point_above_in_robot[0:3]/1000, tcp_actual_pose[3:])
    
        return pose
        
    def go_to_pose(self, pose, acc, vel):
        '''
        Moves the tcp to target pose
        
        arguments:
            - pose (list): robot x,y,z,ry,ry,rz position and orientation
        '''
        self.log.info("--------MOVING TO TARGET POSE--------")
        self.rob.movel(pose, acc, vel, wait=True)
        
    def go_above_point_with_gripper(self, point):
        '''
        Moves the gripper with robot above the detected object
        
        arguments:
            - point (list): object x,y,z position in camera frame
        '''
        point.append(1)
        # Get the target point in tcp frame
        point_in_tcp = np.dot(np.linalg.inv(np.array(self.camera_to_tcp_transform)), np.array(point))
        # Get the target point in gripper frame
        point_in_gripper = np.dot(np.linalg.inv(np.array(self.tcp_to_gripper_transform)), point_in_tcp)
        point_in_gripper[2] -= self.gripper_above_pose_height  # Gripper hight before the picking move in mm
        # Get the translation between the gripper frame and tcp
        gripper_to_tcp_translation = list(transforms3d.affines.decompose44(np.linalg.inv(self.tcp_to_gripper_transform))[0])
        gripper_to_tcp_translation.append(0)
        # Get the tcp target in gripper frame
        tcp_target_in_gripper_frame = np.array(gripper_to_tcp_translation) + np.array(point_in_gripper)
        
        point_above = np.dot(np.array(self.tcp_to_gripper_transform), tcp_target_in_gripper_frame)
        # Get actual position of tcp
        tcp_actual_pose = self.rob.getl()   # get in meter
        # Calculate robot to tcp transformation
        robot_to_tcp_transform = transforms3d.affines.compose(np.array(tcp_actual_pose[0:3])*1000, Rotation.from_rotvec(np.array(tcp_actual_pose[3:])).as_matrix(), [1, 1, 1])
        # Calculate the position for tcp in robot frame, where the gripper is above the object
        point_above_in_robot = np.dot(robot_to_tcp_transform, point_above)
        pose = np.append(point_above_in_robot[0:3]/1000, tcp_actual_pose[3:])
        self.log.info("Move to gripper above: ", pose*1000)

        self.log.info("--------MOVING TO PICK UP POSE--------")
        self.rob.movel(pose, self.above_gripper_acc, self.above_gripper_vel, wait=True)

    def pick(self):
        '''
        Moves down and opens gripper, then moves up to box height.
        '''
        self.rob.movel((0, 0, -self.gripper_above_pose_height/1000, 0, 0, 0), self.pick_down_acc, self.pick_down_vel, wait=True, relative=True) # height is in mm, we convert it to meters for movel()
        self.open_gripper()
        self.rob.movel((0, 0, (self.gripper_above_pose_height+self.box_height)/1000, 0, 0, 0), self.pick_up_acc, self.pick_up_vel, wait=True, relative=True)

    def place(self):
        '''
        Places the object by closing the gripper
        '''
        self.close_gripper()

    def open_gripper(self):
        '''
        Opens gripper
        '''
        self.rob.send_program('set_digital_out(6, True)')
        time.sleep(0.3)
        self.rob.send_program('set_digital_out(6, False)')

    def close_gripper(self): 
        '''
        Closes gripper
        '''  
        self.rob.send_program('set_digital_out(7, True)')
        time.sleep(0.3)
        self.rob.send_program('set_digital_out(7, False)')

    def disconnect(self):
        '''
        Closes the robot connection
        '''
        self.rob.close()

if __name__ == '__main__':
    robot_controller = URController('Configs/config.yaml')
    # robot_controller.move_to_photo_pose()
    print(robot_controller.rob.getj())
    robot_controller.move_to_photo_pose()

        
    robot_controller.disconnect()
