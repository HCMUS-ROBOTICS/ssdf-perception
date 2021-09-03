from torchvision.transforms.transforms import ToTensor
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import torch
from torchvision import transforms

from .segmentation import Model
from .models.mobileunet import MobileUnet

class UnetModel(Model):

    WIN_SHOW_IMAGE = 'Lane Segmentation'

    def on_start(self):
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')
        rospy.loginfo('model device: %s', self.device)

        use_jit = rospy.get_param('~use_jit', default=True)

        model_path = rospy.get_param('~model_path', default=None)

        assert model_path is not None, 'Please provide model path in lane_seg_node'

        if use_jit:
            self.model = torch.jit.load(model_path)
        else:
            self.model = MobileUnet()
            weight = torch.load(model_path, map_location='cpu')
            self.model.load_state_dict(weight)

        self.model = self.model.to(self.device)
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        self.is_show_image = rospy.get_param('~is_show_image', default=False)

    def predict(self, image: CompressedImage):
        np_image = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)

        height, width, _ = np_image.shape

        np_image = cv2.resize(np_image, (224, 224))

        t_input = self.transform(np_image)
        t_input = t_input.unsqueeze(0)
        t_input = t_input.to(self.device)

        with torch.no_grad():
            t_output = self.model(t_input)

        t_output = torch.argmax(t_output, dim=1).float().cpu()
        t_output = t_output.squeeze()

        np_output = t_output.detach().cpu().numpy()

        np_output = cv2.resize(np_output, (width, height))

        if self.is_show_image:
            cv2.imshow(self.WIN_SHOW_IMAGE, np_output)
            cv2.waitKey(1)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', np_output)[1]).tostring()

        return msg