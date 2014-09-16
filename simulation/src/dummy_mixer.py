#!/usr/bin/env python
import rospy
from simulation.msg import mixer_out
from simulation.msg import simout

def callback(data):
    global pub
    msg = mixer_out()
    msg.aileron        = -1
    msg.elevator       = -1
    msg.throttle_0     = 0
    msg.throttle_1     = 0
    msg.throttle_2     = 0
    msg.throttle_3     = 0
    
    pub.publish(msg)

def run_node():
    global pub
    pub = rospy.Publisher("mixer_out",mixer_out,queue_size=10)
    rospy.init_node('mixer_out', anonymous=True)
    
    rospy.Subscriber("simout",simout, callback)
    
    rospy.spin()
   

if __name__ == '__main__':
    run_node()
