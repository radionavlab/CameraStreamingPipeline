#!/usr/bin/python

from __future__ import print_function
import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from camera_streaming_pipeling.srv import RequestRawImages

time_ms = lambda: int(round(time.time() * 1000))
bridge = CvBridge()

def SEIDS(img, scale=0.125):
    # scale the image to 1/8 the resolution
    res = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LANCZOS4)

    # apply high pass filter to original image
    high_pass_orig = cv2.Laplacian(res, cv2.CV_64F)

    # split blue,green and red channels of rescaled image
    b, g, r = cv2.split(res)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

    # split HSV channels of hsv image
    h, s, v = cv2.split(hsv)

    # merge the svb channels
    svb = cv2.merge([s, v, b])

    # apply low pass filter
    gauss_blur = cv2.GaussianBlur(svb, (5, 5), 0)

    # apply high pass filter to blurred image
    high_pass = cv2.Laplacian(gauss_blur, cv2.CV_64F)

    # difference between two high-pass filtered images
    diff = high_pass_orig - high_pass

    # calculate SIEDS value which is standard deviation
    sieds_mean, seids_stddev = cv2.meanStdDev(diff)

    SIEDS, stddev = cv2.meanStdDev(seids_stddev)

    return float(SIEDS)


def request_images(service_name, num_images):
    rospy.wait_for_service(service_name)
    try:
        camera_trigger_proxy = rospy.ServiceProxy(service_name, RequestRawImages)
        return camera_trigger_proxy(num_images).images
    except rospy.ServiceException as e:
        print("Service did not process request: " + str(e))
        return None


def apply_SEIDS(images, scale=0.125):
    return map(lambda img: SEIDS(img, scale), images)


def convert_ros_images_to_cv(ros_images):
    return map(lambda img: bridge.imgmsg_to_cv2(img, "bgr8"), ros_images)


def select_best_image(SEIDS_values, cv_images):
    return cv_images[SEIDS_values.index(max(SEIDS_values))]


def main():
    rospy.init_node('image_quality_selector', anonymous=True)

    raw_image_service_name = rospy.get_param("image_quality_selector/raw_image_service_name")
    num_images = rospy.get_param("image_quality_selector/num_images")

    while True:
        start_time = time_ms()
        ros_images = request_images(raw_image_service_name, num_images)
        cv_images = convert_ros_images_to_cv(ros_images)
        SEIDS_values = apply_SEIDS(cv_images, 1)
        best_image = select_best_image(SEIDS_values, cv_images)
        end_time = time_ms()
        print("Time: " + str(end_time - start_time))
        cv2.imshow('Best Image', best_image)
        cv2.waitKey(0)

if __name__ == '__main__':
    main()
