import copy
import threading
import time
from naoqi import ALProxy
from common import (arm_angle_keys, 
                    angle_thread_lock,
                    time_list
                    )


class RobotProxy(threading.Thread):
    def __init__(self):
        #robotIP = "192.168.1.162"  #Nao
        robotIP = "192.168.1.193"  #Pepper
        port = 9559
        threading.Thread.__init__(self)
        self.motionProxy = ALProxy("ALMotion", robotIP, port)
        self.postureProxy = ALProxy("ALRobotPosture", robotIP, port)
        
        self.motionProxy.wakeUp()
        self.postureProxy.goToPosture("Stand", 2.0)
        self.follow_on = False
        self.angle_list = None
        self.last_tracked_angles = None
    
    def switch_follow_on(self, switch_on=True):
        self.follow_on = switch_on
        
    def stand_return(self):
        self.postureProxy.goToPosture("Stand", 2.0)
    
    def run(self):
        # moniotr how the robot should move
        
        while self.follow_on:
            try:
                angle_thread_lock.acquire()
                if not self.angle_list == self.last_tracked_angles:
                    self.angle_list = self.last_tracked_angles
                    #print("angle_list %s" % str(self.angle_list))
            finally:
                angle_thread_lock.notify_all()
                angle_thread_lock.release()
           
            self.update_robot()
            #time.sleep(0.5)
            
            
    def update_robot(self):
        try:
            if self.angle_list:
                self.motionProxy.angleInterpolation(arm_angle_keys, self.angle_list, time_list, True)
        except:
            pass
            #print("angle_list %s" % str(self.angle_list))
            
        
        
        
