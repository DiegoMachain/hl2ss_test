from unity_robotics_demo_msgs. msg import PosRot, State
import rospy

rospy.init_node('listener', anonymous=True)

def callback(data):
    print(data)

rospy.Subscriber("pos_rot", PosRot, callback)


rospy.spin()

def hook():
    print("Shut down")

rospy.on_shutdown(hook)