#!/usr/bin/env python
#################
"""
Calculating Angle of Objects from the Camera
Duncan Mazza

Adapting code from the example notebook in repository* to run object detection
on video instead of static images. Imports the dictionary using a function I wrote
in pbtxt_converter - takes the argument of LABELS_PATH. Displays image
detection from webcam video feed. Using the camera's field of view, the program
calculates and displays the angle between the center of the frame and the object,
as well as displays a vector from the center of the frame to the center of
the object.

Ross node: /mapping/obstacles
Ross node data structure:
[
 (object_1_name, angle_to_bounding_box_left, angle_to_bounding_box_right),
 (object_2_name, angle_to_bounding_box_left, angle_to_bounding_box_right),
 ... for n objects:
 (object_n_name, angle_to_bounding_box_left, angle_to_bounding_box_right)
]

* https://github.com/NVIDIA-AI-IOT/tf_trt_models
"""

MODEL = 'ssd_mobilenet_v1_coco'
DATA_DIR = '/data/'
CONFIG_FILE = MODEL + '.config'  # ./data/ssd_inception_v2_coco.config
CHECKPOINT_FILE = 'model.ckpt'  # ./data/ssd_inception_v2_coco/model.ckpt
IMAGE_PATH = './data/IMG_5308.jpg'
LABELS_PATH = './data/inception_v2_class_labels.pbtxt'

HORIZONTAL_FIELD_OF_VIEW = 33.28  # number representing the camera's horizontal field of view in degrees
VERTICAL_FIELD_OF_VIEW = 18.72  # number representing the camera's vertical field of view in degrees
VIDEO_SRC = 0  # a value of 0 will engage the webcam
DISPLAY_VIDEO = True

filter_list = ['person', 'boat']
#################

import sys
import cv2
import tensorflow.contrib.tensorrt as trt
import tensorflow as tf
import numpy as np
from tf_trt_models.detection import download_detection_model, build_detection_graph
from tf_trt_models.pbtxt_converter import pbtxt_to_dic
import rospy
import pickle
from std_msgs.msg import Float32, String

# creating ros node for the bounding box and angle data
pub_angle_box = rospy.Publisher('/mapping/obstacles', String, queue_size=10)


def run_detection(dictionary):
    # capturing video from camera, storing in 'cap'. 0 selects the camera
    cap = cv2.VideoCapture(VIDEO_SRC)

    # for displaying text later
    font = cv2.FONT_HERSHEY_COMPLEX

    while True:
        # frame gets the next frame in the camera via cap
        # ret is a boolean for success of capturing frame
        ret, frame = cap.read()

        # initialize list that will contain the ROS data:
        ros_data = []
        #################
        """Load image from the frame, resize, and run network"""
        image = frame
        scores, boxes, classes, num_detections = tf_sess.run(
            [tf_scores,
             tf_boxes,
             tf_classes,
             tf_num_detections],
            feed_dict=
            {
                tf_input: image[None, ...]})

        boxes = boxes[0]  # index by 0 to remove batch dimension
        scores = scores[0]
        classes = classes[0]
        num_detections = num_detections[0]

        image_dims = image.shape
        image_dims = (image_dims[1], image_dims[0])
        cent_frame = (image_dims[0] / 2, image_dims[1] / 2)
        cent_frame_int = (int(cent_frame[0]), int(cent_frame[1]))
        pixels_per_angle = image_dims[0] / HORIZONTAL_FIELD_OF_VIEW
        #################
        # Display rectangles on the image
        for i in range(int(num_detections)):
            # scale box to image coordinates
            box = boxes[i] * np.array([image_dims[0], image_dims[1],
                                       image_dims[0], image_dims[1]])
            ###### Display rectangles using OpenCV
            # Format: image,(topleft_x, topleft_y),(bottomright_x, bottomright_y),color,line thickness
            topleft_x = int(box[1])
            topleft_y = int(box[2])
            bottomright_x = int(box[3])
            bottomright_y = int(box[0])
            # score = round(scores[i], 2)



            # calculations for the vector to the object and the angles
            center_of_square = ((topleft_x + bottomright_x) / 2, (topleft_y + bottomright_y) / 2)
            center_of_square_int = (int(center_of_square[0]), int(center_of_square[1]))

            angle_to_bounding_box_left = (topleft_x - cent_frame_int[0]) / pixels_per_angle
            angle_to_bounding_box_right = (bottomright_x - cent_frame_int[0]) / pixels_per_angle

            if DISPLAY_VIDEO:

                cv2.rectangle(image, (topleft_x, topleft_y),
                              (bottomright_x, bottomright_y),
                              (255, 255, 0), 1, 8, 0)

                # Display angles and vector to object
                cv2.line(image, (cent_frame_int[0], cent_frame_int[1]),
                         (topleft_x, center_of_square_int[1]), (255, 255, 0), 1)
                cv2.line(image, (cent_frame_int[0], cent_frame_int[1]),
                         (bottomright_x, center_of_square_int[1]), (255, 255, 0), 1)

                cv2.putText(image, 'Angle to right: {}'.format(round(angle_to_bounding_box_right, 2)),
                            (center_of_square_int[0], center_of_square_int[1] + 15),
                            font, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(image, 'Angle to left: {}'.format(round(angle_to_bounding_box_left, 2)),
                            (center_of_square_int[0], center_of_square_int[1] + 35),
                            font, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

            # Dictionary contains classifications associated classes[i]
            dictionary_index = classes[i]
            if DISPLAY_VIDEO == True:
                cv2.putText(image, '({}) {}%'.format(dictionary[dictionary_index],
                                                     round(scores[i] * 100), 4), (topleft_x + 5, topleft_y - 7),
                            font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if dictionary[dictionary_index] in filter_list:
                ros_data.append((dictionary[dictionary_index], angle_to_bounding_box_left, angle_to_bounding_box_right))

        if DISPLAY_VIDEO == True:
            # Display crosshaires for reference
            cv2.line(image, (cent_frame_int[0] - 5, cent_frame_int[1]), (cent_frame_int[0] + 5, cent_frame_int[1]),
                     (0, 0, 0), 2)
            cv2.line(image, (cent_frame_int[0], cent_frame_int[1] - 5), (cent_frame_int[0], cent_frame_int[1] + 5),
                     (0, 0, 0), 2)
            # Displaying the image in a window labeled Object Detection
            cv2.imshow('Object Detection', image)

        #########
        # Publish ROS data:
        ros_data_str = pickle.dumps(ros_data)
        pub_angle_box.publish(ros_data_str)
        rate.sleep()
        #########

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('An error occured with OpenCV')
            break
    cap.release()
    cv2.destroyAllWindows()
    # cv2.destroyAllWindows()


if __name__ == "__main__":
    # initialize ROS node
    rospy.init_node("angle_and_bounding_box_video_feed", anonymous=True)  # Initialize nodes
    rate = rospy.Rate(10)  # 10 Hz

    config_path, checkpoint_path = download_detection_model(MODEL, 'data')

    frozen_graph, input_names, output_names = build_detection_graph(
        config=config_path,
        checkpoint=checkpoint_path,
        score_threshold=0.5,
        batch_size=1)

    print(output_names)

    #################
    # Optimize the model with TensorRT
    trt_graph = trt.create_inference_graph(
        input_graph_def=frozen_graph,
        outputs=output_names,
        max_batch_size=1,
        max_workspace_size_bytes=1 << 25,
        precision_mode='FP16',
        minimum_segment_size=50)

    with open('./data/ssd_inception_v2_coco_trt.pb', 'wb') as f:
        f.write(trt_graph.SerializeToString())
    #################

    #################
    # Create session and load graph
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True

    tf_sess = tf.Session(config=tf_config)

    tf.import_graph_def(trt_graph, name='')

    tf_input = tf_sess.graph.get_tensor_by_name(input_names[0] + ':0')
    tf_scores = tf_sess.graph.get_tensor_by_name('detection_scores:0')
    tf_boxes = tf_sess.graph.get_tensor_by_name('detection_boxes:0')
    tf_classes = tf_sess.graph.get_tensor_by_name('detection_classes:0')
    tf_num_detections = tf_sess.graph.get_tensor_by_name('num_detections:0')
    #################

    # Convert the .pbtxt file to a python dictionary
    dictionary = pbtxt_to_dic(LABELS_PATH)
    # Run the object detection; display angles
    try:
        run_detection(dictionary)
    except rospy.ROSInterruptException:
        pass
