import urx
import time

class URController:
    def __init__(self, IP, photo_pose, place_pose):
        self.IP = IP
        self.rob = urx.Robot(self.IP)
        time.sleep(1)
        self.photo_pose = photo_pose
        self.place_pose = place_pose

    def move_to_photo_pose(self):
        self.rob.movej(self.photo_pose, 1, 1, wait=True)
        
    def move_to_place_pose(self):
        self.rob.movej(self.place_pose, 1, 1, wait=True)
        
    def pick(self, pose):
        # DO magic transform
        self.rob.movel(pose, wait=True)
        self.rob.movel((0, 0, -0.1, 0, 0, 0), wait=True, relative=True)
        # DO magic gripper action
        self.rob.movel((0, 0, 0.1, 0, 0, 0), wait=True, relative=True)

    def disconnect(self):
        self.rob.close()

if __name__ == '__main__':
    robot_controller = URController("192.168.1.104",(0, -1.57, 1.57, -1.57, -1.57, 0), (0.2, -1.57, 1.57, -1.57, -1.57, 0))
    robot_controller.move_to_photo_pose()
    pose = robot_controller.rob.getl()
    pose[0] += 0.05
    
    robot_controller.pick(pose)

    robot_controller.move_to_place_pose()
        
    robot_controller.disconnect()

    




