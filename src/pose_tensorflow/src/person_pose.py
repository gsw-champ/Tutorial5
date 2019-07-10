#!/usr/bin/env python3
import os
import sys

import roslib
import rospy

sys.path.append(os.path.dirname(__file__) + "/../")

import numpy as np

from scipy.misc import imread, imsave
from std_msgs.msg import String
from sensor_msgs.msg import Image


from util.config import load_config
from dataset.factory import create as create_dataset
from nnet import predict
from util import visualize
from dataset.pose_dataset import data_to_input

from multiperson.detections import extract_detections
from multiperson.predict import SpatialModel, eval_graph, get_person_conf_multicut
from multiperson.visualize import PersonDraw, visualize_detections

import matplotlib.pyplot as plt

from pose_tensorflow.srv import *
from std_msgs.msg import Float32MultiArray 
# from pose_tensorflow.msg import PersonPose




def callback(req):

    cfg = load_config("src/pose_tensorflow/src/pose_cfg_multi.yaml")
    dataset = create_dataset(cfg)

    sm = SpatialModel(cfg)
    sm.load()

    # pose_person_pub = rospy.Publisher("pose_person", PersonPose, queue_size=1)

    # Load and setup CNN part detector
    sess, inputs, outputs = predict.setup_pose_prediction(cfg)
    rospy.loginfo('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$')

    for i in range(2):
        # Read image from file
        file_name = 'image/a.png'
        image = imread(file_name, mode='RGB')

        image_batch = data_to_input(image)

        # Compute prediction with the CNN
        outputs_np = sess.run(outputs, feed_dict={inputs: image_batch})
        scmap, locref, pairwise_diff = predict.extract_cnn_output(outputs_np, cfg, dataset.pairwise_stats)

        detections = extract_detections(cfg, scmap, locref, pairwise_diff)
        unLab, pos_array, unary_array, pwidx_array, pw_array = eval_graph(sm, detections)
        person_conf_multi = get_person_conf_multicut(sm, unLab, unary_array, pos_array)

        wave_person = []
        for pidx in range(person_conf_multi.shape[0]):

            if person_conf_multi[pidx, 9, 0] != 0 and person_conf_multi[pidx, 9, 1] != 0 and person_conf_multi[pidx, 10, 0] != 0 and person_conf_multi[pidx, 10, 1] != 0:
                
                
                rospy.loginfo('************************************')
                
                
                if (person_conf_multi[pidx, 9, 1] < person_conf_multi[pidx, 5, 1]) or (person_conf_multi[pidx, 10, 1] < person_conf_multi[pidx, 6, 1]):
            
                    rospy.loginfo('########################################')
                    
                    wave_person_x = (person_conf_multi[pidx, 5, 0] + person_conf_multi[pidx, 6, 0]) / 2
                    wave_person_y = (person_conf_multi[pidx, 5, 1] + person_conf_multi[pidx, 6, 1]) / 2
                    
                    wave_person.append([wave_person_x, wave_person_y])

                    img = np.copy(image)

                    visim_multi = img.copy()

                    draw_multi = PersonDraw()
                    fig = plt.imshow(visim_multi)
                    draw_multi.draw(visim_multi, dataset, person_conf_multi)
                    fig.axes.get_xaxis().set_visible(False)
                    fig.axes.get_yaxis().set_visible(False)

                    plt.show()
                    visualize.waitforbuttonpress()


                    if len(wave_person) == 0:
                        rospy.loginfo('No PERSON DETECTED')
                        return False
                    elif len(wave_person) > 1:
                        rospy.loginfo('PERSON DETECTED')
                        id_person = int(len(wave_person)/2)                        
                        person_pose = wave_person[id_person]
                        x = person_pose[0]
                        y = person_pose[1]
                        return SaveImageResponse(x,y)

                    elif len(wave_person) == 1:
                        rospy.loginfo('One PERSON DETECTED')
                        person_pose = wave_person[0]
                        x = person_pose[0]
                        y = person_pose[1]
                        return SaveImageResponse(x,y)

def main():

    service = rospy.Service("person_pose", SaveImage, callback)
    rospy.init_node('pose_tensorflow', anonymous=True)
    rospy.spin()



if __name__ == '__main__':
    main()
