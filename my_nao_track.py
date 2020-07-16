"""
An example that shows how to draw tracked skeletons
It also use joint filtering
"""

import copy
import itertools
import pygame
import pygame.color

from angle_calculator import calculate_arm_angles

from common import arm_angle_keys, angle_thread_lock, robotMirror
from math import degrees
from naoKinect import RobotProxy

from pygame.color import THECOLORS
from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui import SkeletonTrackingState
from pykinect.nui.structs import TransformSmoothParameters

KINECTEVENT = pygame.USEREVENT
WINDOW_SIZE = 800, 600

SKELETON_COLORS = [THECOLORS["red"], 
                   THECOLORS["blue"], 
                   THECOLORS["green"], 
                   THECOLORS["orange"], 
                   THECOLORS["purple"], 
                   THECOLORS["yellow"], 
                   THECOLORS["violet"]]

LEFT_ARM = (JointId.ShoulderCenter, 
            JointId.ShoulderLeft, 
            JointId.ElbowLeft, 
            JointId.WristLeft, 
            JointId.HandLeft)
RIGHT_ARM = (JointId.ShoulderCenter, 
             JointId.ShoulderRight, 
             JointId.ElbowRight, 
             JointId.WristRight, 
             JointId.HandRight)
"""
LEFT_LEG = (JointId.HipCenter, 
            JointId.HipLeft, 
            JointId.KneeLeft, 
            JointId.AnkleLeft, 
            JointId.FootLeft)
RIGHT_LEG = (JointId.HipCenter, 
             JointId.HipRight, 
             JointId.KneeRight, 
             JointId.AnkleRight, 
             JointId.FootRight)
SPINE = (JointId.HipCenter, 
"""

SPINE = (JointId.Head,
         JointId.ShoulderCenter, 
         JointId.Spine )

SMOOTH_PARAMS_SMOOTHING = 0.7
SMOOTH_PARAMS_CORRECTION = 0.4
SMOOTH_PARAMS_PREDICTION = 0.7
SMOOTH_PARAMS_JITTER_RADIUS = 0.1
SMOOTH_PARAMS_MAX_DEVIATION_RADIUS = 0.1
SMOOTH_PARAMS = TransformSmoothParameters(SMOOTH_PARAMS_SMOOTHING,
                                          SMOOTH_PARAMS_CORRECTION,
                                          SMOOTH_PARAMS_PREDICTION,
                                          SMOOTH_PARAMS_JITTER_RADIUS,
                                          SMOOTH_PARAMS_MAX_DEVIATION_RADIUS)

skeleton_to_depth_image = nui.SkeletonEngine.skeleton_to_depth_image




def post_frame(frame):
    """Get skeleton events from the Kinect device and post them into the PyGame
    event queue."""
    try:
        pygame.event.post(
            pygame.event.Event(KINECTEVENT, skeleton_frame=frame)
        )
    except:
        # event queue full
        pass

def get_vector(pointA, pointB):
    vec_x = pointB.x - pointA.x
    vec_y = pointB.y - pointA.y
    vec_z = pointB.z - pointA.z
    
    return [vec_x, vec_y, vec_z]


    
def draw_skeleton_data(dispInfo, screen, pSkelton, index, positions, width = 4, left_arm=True):
    start = pSkelton.SkeletonPositions[positions[0]]
    arm_vec = []
       
    
    return_val = None
    for position in itertools.islice(positions, 1, None):
        joint_id = position.value
        next = pSkelton.SkeletonPositions[joint_id]
        
        curstart = skeleton_to_depth_image(start, dispInfo.current_w, dispInfo.current_h) 
        curend = skeleton_to_depth_image(next, dispInfo.current_w, dispInfo.current_h)

        pygame.draw.line(screen, SKELETON_COLORS[index], curstart, curend, width)
        
        if positions in [RIGHT_ARM, LEFT_ARM]:
            arm_vec.append(get_vector(start, next))
        else:
            return_val = get_vector(start, next)
            
        start = next
        
    if len(arm_vec):
        return_val = calculate_arm_angles(arm_vec, left_arm)
    
    return return_val




def draw_skeletons(dispInfo, screen, skeletons, robot):
    # clean the screen
   
    screen.fill(pygame.color.THECOLORS["black"])

    for index, skeleton_info in enumerate(skeletons):
        # test if the current skeleton is tracked or not
        if skeleton_info.eTrackingState == SkeletonTrackingState.TRACKED:
            # draw the Head
            HeadPos = skeleton_to_depth_image(skeleton_info.SkeletonPositions[JointId.Head], dispInfo.current_w, dispInfo.current_h) 
            spine_vec = draw_skeleton_data(dispInfo, screen, skeleton_info, index, SPINE, 10)
            #pygame.draw.circle(screen, SKELETON_COLORS[index], (int(HeadPos[0]), int(HeadPos[1])), 20, 0)
            
            
                       
            # drawing the limbs
            if robotMirror:
                # human left is robot mirror right
                # human right is robot mirror left
                arm_angles = draw_skeleton_data(dispInfo, screen, skeleton_info, index, RIGHT_ARM, left_arm = True)
                arm_angles += draw_skeleton_data(dispInfo, screen, skeleton_info, index, LEFT_ARM, left_arm=False)      
            else:
                # human left is robot right
                # human right is robot left
                arm_angles = draw_skeleton_data(dispInfo, screen, skeleton_info, index, LEFT_ARM, left_arm=True)      
                arm_angles += draw_skeleton_data(dispInfo, screen, skeleton_info, index, RIGHT_ARM, left_arm = False)
                
            try:
                angle_thread_lock.acquire()
                if robot:
                    robot.last_tracked_angles = arm_angles
                #print("Here 1 %s" % str(robot.last_tracked_angles))
            finally:
                angle_thread_lock.notify_all()
                angle_thread_lock.release()
            """
            fp =open("check4.log", "a")
            fp.write("\n%s : \n" % (str(arm_angle_keys)))
            for i in arm_angles:
              fp.write("%s, " % str(degrees(i)))
            fp.write("\n")
            fp.close()
            """    


def main():
    """Initialize and run the game."""
    pygame.init()    
    robot_following =False
    # Initialize PyGame
    screen = pygame.display.set_mode(WINDOW_SIZE, 0, 16)
    pygame.display.set_caption('PyKinect Skeleton Example')
    screen.fill(pygame.color.THECOLORS["black"])
    #robot = RobotProxy()
    robot=None
    
    try:
        with nui.Runtime() as kinect:
            kinect.skeleton_engine.enabled = True
            kinect.pygame.display.update()
            skeleton_frame_ready += post_frame
    
            # Main game loop
            while True:
                event = pygame.event.wait()
    
                if event.type == pygame.QUIT:
                    break
                elif event.type == KINECTEVENT:
                    # apply joint filtering
                    #kinect._nui.NuiTransformSmooth(event.skeleton_frame, SMOOTH_PARAMS)
    
                    draw_skeletons(pygame.display.Info(), screen, event.skeleton_frame.SkeletonData, robot)
                    
                    if robot and not robot_following:
                        robot_following = True
                        robot.switch_follow_on()
                        robot.start()
    finally:
        if robot :
            if robot_following:
                robot.switch_follow_on(False)
                robot.join()
            robot.stand_return()

if __name__ == '__main__':
    main()
