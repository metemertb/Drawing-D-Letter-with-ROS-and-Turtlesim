import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
pose = None
def callbackPose(msg):
    global pose
    pose = msg

def move(lin_vel, distance):
    while pose is None:
        rospy.sleep(0.01)

    x0 = pose.x
    y0 = pose.y
    print("Start Position: ", x0, y0)
    vel_msg = Twist()

    while True:

        dist = math.sqrt((pose.x - x0) ** 2 + (pose.y - y0) ** 2)
        print("Distance Traveled: ", dist)

        if dist < 0.99 * distance:
            vel_msg.linear.x = lin_vel
            vel_pub.publish(vel_msg)
        else:
            vel_msg.linear.x = 0
            vel_pub.publish(vel_msg)
            break
        loop_rate.sleep()


def rotate(angl_vel, angle):
    while pose is None:
        rospy.sleep(0.01)
    vel_msg = Twist()

    while True:
        diff = abs(pose.theta - angle)
        print("Angle Difference: ", diff)

        if diff > 0.01:
            vel_msg.angular.z = angl_vel
            vel_pub.publish(vel_msg)
        else:
            vel_msg.angular.z = 0
            vel_pub.publish(vel_msg)
            break

        loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('motion', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callbackPose)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)
    loop_rate = rospy.Rate(100)
    for i in range(4):
        rotate(0.5,math.pi)
        move(0.5, 0.5)
        rotate(0.5, -math.pi / 2)
        move(0.5, 1.5)
        rotate(0.5, 0)
        move(0.5, 0.5)
        for i in range(17):
            rotate(0.1, (math.pi / 18)* (i+1))
            move(0.5, 0.130734)