"""
An example that shows how to draw tracked skeletons
It also use joint filtering
"""

import itertools
import pygame
import pygame.color

from pygame.color import THECOLORS
from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui import SkeletonTrackingState
from pykinect.nui.structs import TransformSmoothParameters
from math import acos, asin, atan2, degrees, pi, sqrt 
from numpy import dot, cross
from numpy.linalg.linalg import norm

KINECTEVENT = pygame.USEREVENT
WINDOW_SIZE = 800, 600

SKELETON_COLORS = [THECOLORS["red"], 
                   THECOLORS["blue"], 
                   THECOLORS["green"], 
                   THECOLORS["orange"], 
                   THECOLORS["purple"], 
                   THECOLORS["yellow"], 
                   THECOLORS["violet"]]

LEFT_ARM = (JointId.Spine,
            JointId.ShoulderCenter, 
            JointId.ShoulderLeft, 
            JointId.ElbowLeft, 
            JointId.WristLeft, 
            JointId.HandLeft)
RIGHT_ARM = (JointId.Spine,
             JointId.ShoulderCenter, 
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

SPINE = (JointId.Spine, 
         JointId.ShoulderCenter, 
         JointId.Head)

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

class QuarternionRotation(object):
    def __init__(self, rotAxis, rotAngle):
        self.w = rotAngle
        self.x = rotAxis[0]
        self.y = rotAxis[1]
        self.z = rotAxis[2]
    
    def __str__(self):
        
        return "Axis = [%s, %s, %s]  Rotation =%s" % (self.x,
                                                      self.y,
                                                      self.z,
                                                     degrees(acos(self.w)))
    
    
    def Quaternion_toEulerianAngle(self):
        q = self
        qx2 = q.x * q.x
        qy2 = q.y * q.y
        qz2 = q.z * q.z
        qw2 = q.w * q.w
        
        t0 = 2.0 * (q.w * q.z + q.x*q.y)
        t1 = 1.0 - 2.0 * (qz2 + qx2)
        roll = degrees(atan2(t0, t1))
        
        t2 = 2.0 * (q.w*q.x - q.y*q.z)
        t2 =  1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        pitch = degrees(asin(t2))
        
        t3 = 2.0 * (q.w * q.y + q.z*q.x)
        t4 = 1.0 - 2.0 * (qx2 + qy2)
        yaw = degrees(atan2(t3, t4))
        
        
        return [roll, pitch, yaw] 
    
    def quaternion2euler(self):
        q = self
        qx2 = q.x * q.x
        qy2 = q.y * q.y
        qz2 = q.z * q.z
        qw2 = q.w * q.w
        
        
        test = q.x*q.y + q.z*q.w
                
        if (test > 0.499):
            roll    = atan2(q.x,q.w)
            pitch = pi/2
            yaw     = 0
        elif (test < -0.499):
            roll    = atan2(q.x,q.w)
            pitch = pi/2
            yaw     = 0
        else:
            roll = atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * qy2 - 2 * qz2)
            pitch = asin(2*q.x*q.y+2*q.z*q.w)
            yaw = atan2(2*q.x*q.w-2*q.y*q.z,1-2*qx2-2*qz2)
        """
        
        roll = arctan2(q.x*qz + q.w*q.y, q.x*q.w - q.y*q.z)
        pitch = arccos(-qx2 -qy2 + qz2 + qw2)
        yaw = arctan2(qx*qz - qy*qw, qy*qz + qx*qw) 
        """
        return [degrees(roll), degrees(pitch), degrees(yaw)]

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
    
    vec_len = sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z)
    
    if vec_len:
        unit_x = vec_x/vec_len
        unit_y = vec_y/vec_len
        unit_z = vec_z/vec_len
    else:
        unit_x = vec_x
        unit_y = vec_y
        unit_z = vec_z
    
    return [unit_x, unit_y, unit_z]
    
    
def draw_skeleton_data(dispInfo, screen, pSkelton, index, positions, width = 4):
    start = pSkelton.SkeletonPositions[positions[0]]
    arm_vec = []
    
       
    
    return_val = (None, None)
    for position in itertools.islice(positions, 1, None):
        joint_id = position.value
        next = pSkelton.SkeletonPositions[joint_id]
        
        curstart = skeleton_to_depth_image(start, dispInfo.current_w, dispInfo.current_h) 
        curend = skeleton_to_depth_image(next, dispInfo.current_w, dispInfo.current_h)

        pygame.draw.line(screen, SKELETON_COLORS[index], curstart, curend, width)
        
        if positions in [LEFT_ARM, RIGHT_ARM]:
            arm_vec.append(get_vector(start, next))            
            
        start = next
        
    if len(arm_vec):
        angle_val = None
        quart_vec = None
        
        quart_rot = []
        base_vec = arm_vec[0]        
        for next_vec in itertools.islice(arm_vec, 2, 4):
            angle_val = dot(base_vec, next_vec)
            q = cross(base_vec, next_vec)
            q_len = norm(q)
            if q_len:
                quart_vec = q/q_len
            else:
                quart_vec = q 
            quart_rot.append(QuarternionRotation(quart_vec, angle_val))
            
            base_vec = next_vec
        fp =open("check6.log", "a")
        i=1
        for q in quart_rot:
            fp.write(str(q))
            fp.write('\n')
            arm_part = "wrist"
            if i==1:
                arm_part = "shoulder"
            elif i==2:
                arm_part = "elbow"                
            fp.write("%s Eular Angle : %s\n" % (arm_part, str(q.quaternion2euler())))
            fp.write("%s Eular Angle New : %s\n" % (arm_part, str(q.Quaternion_toEulerianAngle())))
            fp.write('\n')
            i+=1
        fp.close()
        return_val = (quart_vec, angle_val)
        
    
    return return_val




def draw_skeletons(dispInfo, screen, skeletons):
    # clean the screen
    screen.fill(pygame.color.THECOLORS["black"])
    myfont = pygame.font.SysFont('Comic Sans MS', 30)
    for index, skeleton_info in enumerate(skeletons):
        # test if the current skeleton is tracked or not
        if skeleton_info.eTrackingState == SkeletonTrackingState.TRACKED:
            # draw the Head
            HeadPos = skeleton_to_depth_image(skeleton_info.SkeletonPositions[JointId.Head], dispInfo.current_w, dispInfo.current_h) 
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, SPINE, 10)
            pygame.draw.circle(screen, SKELETON_COLORS[index], (int(HeadPos[0]), int(HeadPos[1])), 20, 0)
            
            # drawing the limbs
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, LEFT_ARM)
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, RIGHT_ARM)
            
            
            HipPos = skeleton_info.SkeletonPositions[JointId.HipCenter]
            distance = "%.2f" % HipPos.z
            textsurface = myfont.render(distance, False, (0, 255, 0))
            
            HipPos_screen = skeleton_to_depth_image(HipPos, dispInfo.current_w, dispInfo.current_h)
            screen.blit(textsurface,(int(HipPos_screen[0]), int(HipPos_screen[1])))
            
            
            
            
                


def main():
    """Initialize and run the game."""
    pygame.init()
    pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.


    # Initialize PyGame
    screen = pygame.display.set_mode(WINDOW_SIZE, 0, 16)
    pygame.display.set_caption('PyKinect Skeleton Example')
    screen.fill(pygame.color.THECOLORS["black"])

    with nui.Runtime() as kinect:
        kinect.camera.elevation_angle = 0
        kinect.skeleton_engine.enabled = True
        kinect.skeleton_frame_ready += post_frame

        # Main game loop
        while True:
            event = pygame.event.wait()

            if event.type == pygame.QUIT:
                break
            elif event.type == KINECTEVENT:
                # apply joint filtering
                kinect._nui.NuiTransformSmooth(event.skeleton_frame, SMOOTH_PARAMS)

                draw_skeletons(pygame.display.Info(), screen, event.skeleton_frame.SkeletonData)
                pygame.display.update()
                pass

if __name__ == '__main__':
    main()
