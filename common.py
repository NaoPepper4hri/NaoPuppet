import threading

arm_angle_keys = ['LShoulderPitch', 
                  'LShoulderRoll', 
                  'LElbowYaw', 
                  'LElbowRoll',  
                  'RShoulderPitch', 
                  'RShoulderRoll',  
                  'RElbowYaw', 
                  'RElbowRoll' ]
time_list = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

robotMirror = True

#last_tracked_angles = None
angle_thread_lock = threading.Condition()