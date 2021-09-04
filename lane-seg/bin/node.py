#!/usr/bin/python3

import rospy
from lane_seg.segmentation import Segmentation
from lane_seg.unet_model import UnetModel

def main():
    rospy.init_node('lane_seg_node')
    rospy.loginfo('lane_seg_node is initialized')

    segmentation = Segmentation()

    segmentation.set_model(UnetModel())

    segmentation.run_loop()

if __name__=='__main__':
    main()