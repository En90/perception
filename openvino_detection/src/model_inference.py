#!/home/en/openvino_env/bin/python3

from openvino.runtime import Core
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import numpy as np
import rospy
import cv2
import ipywidgets as widgets

class Node:
    def __init__(self):
        rospy.init_node('model_inference',anonymous=False,log_level=rospy.INFO)
        self.__core = Core()
        self.__bridge = CvBridge()
        self.__device = widgets.Dropdown(
            options=self.__core.available_devices + ["AUTO"],
            value='AUTO',
            description='Device:',
            disabled=False,
        )
        
        if rospy.has_param("~model_thresh"):
            self.__model_thresh = rospy.get_param("~model_thresh")
        else:
            self.__model_thresh = 0.8
            
        if rospy.has_param('~model_path'):
            model_path_ = rospy.get_param("~model_path")
        else:
            rospy.logerr("Need to specify model path")
            
        if rospy.has_param('~filter_bbox_ratio_max'):
            self.__filter_size_ratio_max = rospy.get_param("~filter_bbox_ratio_max")
        else:
            self.__filter_size_ratio_max = 1
            
        if rospy.has_param('~filter_bbox_ratio_min'):
            self.__filter_size_ratio_min = rospy.get_param("~filter_bbox_ratio_min")
        else:
            self.__filter_size_ratio_min = 0
        
        self.__input_key_de, self.__output_keys_de, self.__compiled_model_de  = self.__model_init(model_path_)
        self.__image_sub = rospy.Subscriber("/input_image", Image, self.__image_callback)
        self.__image_pub = rospy.Publisher("/result", Image, queue_size=5)
        self.__position_pub = rospy.Publisher("/pixel_position", PointStamped, queue_size=10)
        
    def __model_init(self, model_path: str) -> tuple:
        """
        Read the network and weights from file, load the
        model on the CPU and get input and output names of nodes

        :param: model: model architecture path *.xml
        :retuns:
                input_key: Input node network
                output_key: Output node network
                exec_net: Encoder model network
                net: Model network
        """
        # Read the network and corresponding weights from a file.
        model = self.__core.read_model(model=model_path)
        compiled_model = self.__core.compile_model(model=model, device_name=self.__device.value)
        # Get input and output names of nodes.
        input_keys = compiled_model.input(0)
        output_keys = compiled_model.output(0)
        return input_keys, output_keys, compiled_model
    
    def __image_callback(self, msg):
        # Get input size - Detection.
        height_de, width_de = list(self.__input_key_de.shape)[2:]
        h: Header = msg.header
        try:
            image = self.__bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            # Resize it to [3, 256, 256].
            resized_image_de = cv2.resize(image, (width_de, height_de))
            # Expand the batch channel to [1, 3, 256, 256].
            input_image_de = np.expand_dims(resized_image_de.transpose(2, 0, 1), 0)
        except CvBridgeError as e:
            rospy.logwarn(e)
        # Run inference.
        boxes = self.__compiled_model_de([input_image_de])[self.__output_keys_de]
        # Delete the dim of 0, 1.
        boxes = np.squeeze(boxes, (0, 1))
        # Remove zero only boxes.
        boxes = boxes[~np.all(boxes == 0, axis=1)]
        
        car_points = list()
        result_image = self.__convert_result_to_image(image, input_image_de, boxes, car_points, threshold=self.__model_thresh)
        try:
            image_message = self.__bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
            # image_message = self.__bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        image_message.header = h
        self.__image_pub.publish(image_message)
        if(len(car_points)):
            for p in car_points:
                p.header = h
                self.__position_pub.publish(p)
        else:
            pass

    
    def __crop_images(self, bgr_image, resized_image, boxes, threshold=0.8) -> np.ndarray:
        """
        Use bounding boxes from detection model to find the absolute car position

        :param: bgr_image: raw image
        :param: resized_image: resized image
        :param: boxes: detection model returns rectangle position
        :param: threshold: confidence threshold
        :returns: car_position: car's absolute position
        """
        # Fetch image shapes to calculate ratio
        (real_y, real_x), (resized_y, resized_x) = bgr_image.shape[:2], resized_image.shape[:2]
        ratio_x, ratio_y = real_x / resized_x, real_y / resized_y

        # Find the boxes ratio
        boxes = boxes[:, 2:]
        # Store the vehicle's position
        car_position = []
        # Iterate through non-zero boxes
        for box in boxes:
            # Pick confidence factor from last place in array
            conf = box[0]
            if conf > threshold:
                # Convert float to int and multiply corner position of each box by x and y ratio
                # In case that bounding box is found at the top of the image,
                # upper box  bar should be positioned a little bit lower to make it visible on image
                (x_min, y_min, x_max, y_max) = [
                    int(max(corner_position * ratio_y * resized_y, 10)) if idx % 2
                    else int(corner_position * ratio_x * resized_x)
                    for idx, corner_position in enumerate(box[1:])
                ]
                
                # filter by bbox area
                bbox_size = (x_max - x_min) * (y_max - y_min)
                image_size = real_x * real_y
                if bbox_size > (image_size * self.__filter_size_ratio_min) and bbox_size < (image_size * self.__filter_size_ratio_max):
                    car_position.append([x_min, y_min, x_max, y_max, conf])

        return car_position
    
    def __convert_result_to_image(self, raw_image, resized_image, boxes, pixel_positions: list, threshold=0.8):
        """
        Use Detection model boxes to draw rectangles and plot the result

        :param: compiled_model_re: recognition net
        :param: input_key_re: recognition input key
        :param: raw image
        :param: resized_image: resized image
        :param: boxes: detection model returns rectangle position
        :param: threshold: confidence threshold
        :returns: rgb_image: processed image
        """
        # Define colors for boxes and descriptions.
        colors = {"red": (255, 0, 0), "green": (0, 255, 0)}

        # Convert the base image from BGR to RGB format.
        rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)

        # Find positions of cars.
        car_position = self.__crop_images(raw_image, resized_image, boxes, threshold)

        for x_min, y_min, x_max, y_max, conf in car_position:
            # Draw a bounding box based on position.
            # Parameters in the `rectangle` function are: image, start_point, end_point, color, thickness.
            rgb_image = cv2.rectangle(rgb_image, (x_min, y_min), (x_max, y_max), colors["red"], 2)
            p = PointStamped()
            p.point.x = (x_max + x_min)/2
            p.point.y = (y_min + y_max)/2
            p.point.z = conf
            pixel_positions.append(p)
            rgb_image = cv2.circle(rgb_image, (round((x_max + x_min)/2), round((y_min + y_max)/2)), 2, colors["green"], 2)
            # Print the attributes of a vehicle.
            # Parameters in the `putText` function are: img, text, org, fontFace, fontScale, color, thickness, lineType.
            # rgb_image = cv2.putText(
            #     rgb_image,
            #     f"{attr_color} {attr_type}",
            #     (x_min, y_min - 10),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     2,
            #     colors["green"],
            #     10,
            #     cv2.LINE_AA
            # )

        return rgb_image
    
if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")