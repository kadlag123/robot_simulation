import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from digitizer.cfg import DigitizerConfig
from dynamic_reconfigure.server import Server

class ros(object):
    def __init__(self):
        rospy.init_node('digitizer')
        self.param = dict()
        srv = Server(LineParamConfig, self.reconfig)

        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.msg = Twist()
        self.bridge = CvBridge()

        self.intg = self.lastError = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        error = self.process(cv_image)
        balance = self.pid(error)
        self.msg.linear.x = self.param['SP']
        self.msg.angular.z = balance
        self.pub.publish(self.msg)
        self.rate.sleep()

    def reconfig(self, config, level):
        self.param = config
        return config

    def pid(self, error):
        print(error)
        prop = error
        self.intg -= error
        diff = error - self.lastError
        self.lastError = error
        balance = self.param['KP'] * prop + self.param['KI'] * self.intg + self.param['KD'] * diff
        return balance
