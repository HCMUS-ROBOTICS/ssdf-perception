#!/usr/bin/python3

import rospy
from lane_segmentation.segmentation import Segmentation
from lane_segmentation.unet_model import UnetModel

def main():
    rospy.init_node('lane_segmentation_node')
    rospy.loginfo('lane_segmentation_node is initialized')

    segmentation = Segmentation()

    segmentation.set_model(UnetModel())

    segmentation.run_loop()

if __name__=='__main__':
    main()