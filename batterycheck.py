import rospy
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from time import sleep
from std_msgs.msg import Empty


def callback(data):
	rospy.loginfo("Battery level is %s",data.percent)
	print(data.percent)

	if data.percent<30:
		rate = rospy.Rate(10)
		rospy.loginfo("Battery is below 30%. Landing now.")
		while rospy.is_shutdown():
			pub = rospy.Publisher('/bebop/land', std_msgs/Empty, queue_size=0)
			pub.publish()
			rate.sleep()

def main():
	rospy.init_node('Battery_Level')
	rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, callback)
	rospy.spin()

if __name__ == '__main__':
	main()