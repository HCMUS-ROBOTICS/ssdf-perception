# Lane Detection

This package perform lane detection


# Installation

Updating...


# Package's inputs/outputs

## Publishing topics

This package will publish to the topic set by `~pub_lane_topic` argument. Its type is `ssdf_msgs/Lane`.
Please have a look on `ssdf-msgs` repository for more detail.

## Subscribing topics

It depends on the implementation used in this class. Currently, we only support sliding window algorithm.
However, it might be updated in the future.

1. Sliding window

   a lane line segmentation image published on `~sub_laneseg`. The transport hint `compressed` is used.

1. Updating...

# Running

For example, if we want to run lane extraction on lane line segmentation images published on `/lane_seg/compressed` topic,
and we want to return lane parameters on `/lane` topic using `compressed` image transport, we could run the command as follow.

```
rosrun lane_extraction lane_extraction_node _image_transport:=compressed _sub_laneseg:=/lane_seg _pub_lane_topic:=/lane
```
