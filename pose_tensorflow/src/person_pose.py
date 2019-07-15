#!/usr/bin/env python3
import os
import sys

import roslib
import rospy

sys.path.append(os.path.dirname(__file__) + "/../")

import numpy as np

from scipy.misc import imread, imsave
from scipy.ndimage import filters
# from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image

from util.config import load_config
from dataset.factory import create as create_dataset
from nnet import predict
from util import visualize
from dataset.pose_dataset import data_to_input

from multiperson.detections import extract_detections
from multiperson.predict import SpatialModel, eval_graph, get_person_conf_multicut
from multiperson.visualize import PersonDraw, visualize_detections

import matplotlib.pyplot as plt

ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

from pose_tensorflow.msg import *
from perception_person.srv import *


# class image_to_pose():

#     def __init__(self):
#         # initialze pub and sub
#         self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)

#         self.image_sub = rospy.Subscriber("xtion/rgb/image_raw/compressed", \
#             CompressedImage, self.callback, queue_size=1)
#         print('image recieving')

def callback():

    cfg = load_config("src/pose_tensorflow/src/pose_cfg_multi.yaml")
    dataset = create_dataset(cfg)

    sm = SpatialModel(cfg)
    sm.load()

    draw_multi = PersonDraw()

    zero_np = np.zeros([1,2])

    image_pose_pub = rospy.Publisher("/output/person_pose", PersonPose, queue_size=1)
    person_pose_client = rospy.ServiceProxy("position_base/point3D", PercepPerson)
    image_pub = rospy.Publisher("/output/image/compressed", CompressedImage, queue_size=1)
    msg = PersonPose()
    msg_image = CompressedImage()

    # pose_person_pub = rospy.Publisher("pose_person", PersonPose, queue_size=1)

    # Load and setup CNN part detector
    sess, inputs, outputs = predict.setup_pose_prediction(cfg)
    print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$')

    # person_x_old = 0


    while not rospy.is_shutdown():

        rospy.loginfo("Getting image...")
        img = rospy.wait_for_message("/xtion/rgb/image_raw/compressed", CompressedImage)
        rospy.loginfo("Got image!")
        # img = rospy.wait_for_message("xtion/rgb/image_raw/compressed", CompressedImage)
        np_arr = np.fromstring(img.data, np.uint8)
        # np arrary image
        image = cv2.imdecode(np_arr, 1)

        # print('################',image.shape) 480, 640, 3
        # # Read image from file cv2.CV_LOAD_IMAGE_COLOR
        # file_name = 'image/a.png'
        # image = imread(file_name, mode='RGB')

        image_batch = data_to_input(image)

        # Compute prediction with the CNN
        outputs_np = sess.run(outputs, feed_dict={inputs: image_batch})
        scmap, locref, pairwise_diff = predict.extract_cnn_output(
            outputs_np, cfg, dataset.pairwise_stats)

        detections = extract_detections(cfg, scmap, locref, pairwise_diff)
        unLab, pos_array, unary_array, pwidx_array, pw_array = eval_graph(
            sm, detections)
        person_conf_multi = get_person_conf_multicut(
            sm, unLab, unary_array, pos_array)

        # draw joint


        # img = np.copy(image)

        # visim_multi = img.copy()

        # fig = plt.imshow(visim_multi)
        draw_multi.draw(image, dataset, person_conf_multi)
        # fig.axes.get_xaxis().set_visible(False)
        # fig.axes.get_yaxis().set_visible(False)

        # plt.show()
        # visualize.waitforbuttonpress()

        wave_person = []
        for pidx in range(person_conf_multi.shape[0]):

            # cv2.circle(image, (int(person_conf_multi[pidx, 9, 0]),int(person_conf_multi[pidx, 9, 1])), 1, (255,255,255), -1)
            # cv2.circle(image, (int(person_conf_multi[pidx, 10, 0]),int(person_conf_multi[pidx, 10, 1])), 1, (255,255,255), -1)

            # shoudler, elbow, wrist
            if (person_conf_multi[pidx, 5, 0] != 0 and person_conf_multi[pidx, 5, 1] != 0 \
                and person_conf_multi[pidx, 6, 0] != 0 and person_conf_multi[pidx, 6, 1] != 0 \
                and person_conf_multi[pidx, 7, 0] != 0 and person_conf_multi[pidx, 7, 1] != 0 \
                and person_conf_multi[pidx, 8, 0] != 0 and person_conf_multi[pidx, 8, 1] != 0 \
                and person_conf_multi[pidx, 9, 0] != 0 and person_conf_multi[pidx, 9, 1] != 0 \
                and person_conf_multi[pidx, 10, 0] != 0 and person_conf_multi[pidx, 10, 1] != 0): 

                if (person_conf_multi[pidx, 9, 1] < person_conf_multi[pidx, 5, 1]) or \
                    (person_conf_multi[pidx, 10, 1] < person_conf_multi[pidx, 6, 1]):
                    
                    # if person_x_old == 0:
                    #     person_x_old = person_conf_multi[pidx, 9, 0]
                    # else:
                    #     while abs(person_conf_multi[pidx, 9, 0] - person_x_old) > 50:

                    #         person_x_old = person_conf_multi[pidx, 9, 0]


                    wave_person_x = (person_conf_multi[pidx, 5, 0] + person_conf_multi[pidx, 6, 0]) / 2
                    wave_person_y = (person_conf_multi[pidx, 5, 1] + person_conf_multi[pidx, 6, 1]) / 2

                    wave_person.append([wave_person_x, wave_person_y])

                    if len(wave_person) == 0:
                        rospy.loginfo('No PERSON DETECTED')
                        return False

                    elif len(wave_person) > 1:
                        rospy.loginfo('PERSON DETECTED')
                        id_person = int(len(wave_person) / 2)
                        person_pose = wave_person[id_person]
                        person_pose = np.around(person_pose, decimals=3)
                        x = person_pose[0]
                        y = person_pose[1]

                        msg.x = x
                        msg.y = y
                        image_pose_pub.publish(msg)
                        point3D = person_pose_client(x, y)
                        
                        cv2.putText(image, str(person_pose), (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)


                    elif len(wave_person) == 1:
                        rospy.loginfo('One PERSON DETECTED')
                        person_pose = wave_person[0]
                        person_pose = np.around(person_pose, decimals=3)
                        x = person_pose[0]
                        y = person_pose[1]

                        msg.x = x
                        msg.y = y
                        image_pose_pub.publish(msg)
                        person_pose_client(x, y)

                        cv2.putText(image, str(person_pose), (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)

        cv2.imshow('show', image)
        cv2.waitKey(2)




                    


def main():
    rospy.init_node('pose_tensorflow')
    callback()    
    rospy.spin()


if __name__ == '__main__':
    main()
