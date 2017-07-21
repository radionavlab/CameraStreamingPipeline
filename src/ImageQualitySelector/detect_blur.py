# import the necessary packages
from __future__ import print_function

import roslib

roslib.load_manifest('detect_blur')
import sys
import rospy
from imutils import paths
import argparse
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def variance_of_laplacian(Image):
    # compute the Laplacian of the image and then return the focus
    # measure, which is simply the variance of the Laplacian
    return cv2.Laplacian(Image, cv2.CV_64F).var()


# detect how much blur in picture subscribed from stream
# publish images with blur less than threshold
class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/snap_cam_highres_publisher/UnBlurryImage", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/snap_cam_highres_publisher/image", Image, self.callback)

    def callback(self, data):
        # convert image message from ros to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridfeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

            # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-t", "--threshold", type=float, default=150.0,
                        help="focus measures that fall below this value will be considered 'blurry'")
        args = vars(ap.parse_args())

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        fm = variance_of_laplacian(gray)
        text = "Not Blurry"

        # if the focus measure is less than the supplied threshold,
        # then the image should be considered "blurry"
        if fm < args["threshold"]:
            text = "Blurry"

            # show the image
        cv2.putText(cv_image, "{}: {:.2f}".format(text, fm), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        print("hello")

        # convert OpenCV image to Ros Message image
        if fm >= args["threshold"]:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
