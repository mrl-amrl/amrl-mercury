import rospy
from mercury import Joy

if __name__ == "__main__":
    rospy.init_node("mercury_joy", anonymous=False)

    def on_pressed():
        rospy.logwarn('pressed')
    
    def on_changed(value):
        rospy.logwarn(value)

    controller = Joy()
    controller.on_pressed(rospy.get_param('~button'), on_pressed)
    controller.on_changed(rospy.get_param('~axes'), on_changed)
    controller.spin()
