#!/usr/bin/env python3
from __future__ import division
import os
import sys

import numpy as np

print('######', sys.path)
sys.path.append(os.path.dirname(__file__) + "/../")

from scipy.misc import imread, imsave

print('#######', sys.path)
from util.config import load_config
from dataset.factory import create as create_dataset
from nnet import predict
from util import visualize
from dataset.pose_dataset import data_to_input

from multiperson.detections import extract_detections
from multiperson.predict import SpatialModel, eval_graph, get_person_conf_multicut
from multiperson.visualize import PersonDraw, visualize_detections

import matplotlib.pyplot as plt


cfg = load_config("demo/pose_cfg_multi.yaml")
dataset = create_dataset(cfg)

sm = SpatialModel(cfg)
sm.load()

draw_multi = PersonDraw()

# Load and setup CNN part detector
sess, inputs, outputs = predict.setup_pose_prediction(cfg)

for i in range(10):
    # Read image from file
    file_name = "/home/atHomeSS19/mao_ws/workspace/image/{}.png".format(i)
    image = imread(file_name, mode='RGB')
    print('#######', type(image))

    image_batch = data_to_input(image)

    # Compute prediction with the CNN
    outputs_np = sess.run(outputs, feed_dict={inputs: image_batch})
    scmap, locref, pairwise_diff = predict.extract_cnn_output(outputs_np, cfg, dataset.pairwise_stats)

    detections = extract_detections(cfg, scmap, locref, pairwise_diff)
    unLab, pos_array, unary_array, pwidx_array, pw_array = eval_graph(sm, detections)
    person_conf_multi = get_person_conf_multicut(sm, unLab, unary_array, pos_array)

    wave_person = dict()
    for pidx in range(person_conf_multi.shape[0]):

        if person_conf_multi[pidx, 9, 0] != 0 and person_conf_multi[pidx, 9, 1] != 0 and person_conf_multi[pidx, 10, 0] != 0 and person_conf_multi[pidx, 10, 1] != 0:

            if (person_conf_multi[pidx, 9, 1] < person_conf_multi[pidx, 5, 1]) or (person_conf_multi[pidx, 10, 1] < person_conf_multi[pidx, 6, 1]):

                wave_person_x = (person_conf_multi[pidx, 5, 0] + person_conf_multi[pidx, 6, 0]) / 2
                wave_person_y = (person_conf_multi[pidx, 5, 1] + person_conf_multi[pidx, 6, 1]) / 2
                
                wave_person[pidx] = [wave_person_x, wave_person_y]
                


print(wave_person)

img = np.copy(image)

visim_multi = img.copy()

fig = plt.imshow(visim_multi)
draw_multi.draw(visim_multi, dataset, person_conf_multi)
fig.axes.get_xaxis().set_visible(False)
fig.axes.get_yaxis().set_visible(False)

plt.show()
visualize.waitforbuttonpress()
