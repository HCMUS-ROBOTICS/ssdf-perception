import rospy
from sensor_msgs.msg import CompressedImage


class Model():
    """This class is to wrap any segmentation model from libraries such as pytorch, tensorflow."""

    def on_start(self):
        r"""Start callback.

        This method is being called before the rospy.spin loop.

        It is necessary for some systems that they need to initialize stuffs before
        the spinning loop starts.
        """
        pass

    def on_stop(self):
        r"""Exit callback.

        This method is being called after the rospy.spin loop.

        It is necessary for some systems that they need to release stuffs after
        the spinning loop ends.
        """
        pass

    def predict(self, image: CompressedImage):
        r"""Predict function of the segmentation model.

        Args:
            image: "CompressedImage" from camera
        Returns:
            seg: "CompressedImage" of the prediction
        """
        pass


class Segmentation():
    def __init__(self) -> None:

        rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image/compressed')
        seg_topic = rospy.get_param('~seg_topic', '/camera/seg/image/compressed')

        queue_size = rospy.get_param('~queue_size', default=10)

        self.rgb_sub = rospy.Subscriber(rgb_topic, CompressedImage, self.callback_rgb_image,
                                        queue_size=1, buff_size=2**24)
        self.seg_pub = rospy.Publisher(seg_topic, CompressedImage, queue_size=queue_size)

    def set_model(self, model: Model):
        r"""Set model for this segmentation.

        Args:
            model: an instance of "Model"
        """
        self.model = model

    def callback_rgb_image(self, image: CompressedImage):
        r"""RGB image callback.

        This callback will publish the segmentation result to `seg_topic`

        Args:
            image: "CompressedImage" from camera
        """
        self.seg_pub.publish(self.model.predict(image))

    def run_loop(self):
        r"""Run main loop.

        This will hang the process to receive as well as publish messages via callbacks
        initialized from the constructor.
        """
        self.model.on_start()
        rospy.spin()
        self.model.on_stop()
