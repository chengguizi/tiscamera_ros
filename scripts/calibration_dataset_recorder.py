
#!/usr/bin/env python2


import __future__


import time
from datetime import datetime

import numpy as np
import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from threading import Lock, Thread
import queue

class ApproximateSyncImageSubscriber:
    
    def __init__(self, image_topics):
        self.subscribers = {}
        self.queues = {} # store individual incoming images
        self.latest = None
        self.size = len(image_topics)

        self.rolling = False
        self.assemble = {}
        self.assemble_index = -1

        self.lock = Lock()

        for topic in image_topics:
            print("subscribing to image topic {}".format(topic))
            self.subscribers[topic] = rospy.Subscriber(topic, Image, self.image_callback, callback_args=topic)
            self.queues[topic] = queue.Queue(50)

    def image_callback(self, msg, topic_name):
        seq = msg.header.seq
        # print("{} image received, seq = {}".format(topic_name, seq))

        self.queues[topic_name].put(msg)

        with self.lock:
            self.queue_update()

    def queue_update(self):

        for queueName in self.queues:

            m_queue = self.queues[queueName]

            # already in assemble, no need to get from queue
            if queueName in self.assemble:
                continue

            while True:

                if m_queue.empty():
                    return

                imageMsg = m_queue.get()

                ts = imageMsg.header.stamp.to_sec()
                # image = imageMsg[1]

                if self.assemble_index < ts - 1.0/10:
                    # we shall throw away the assemble and start again
                    
                    if self.assemble_index != -1:
                        print("reset index to {}".format(ts))

                    self.assemble_index = ts
                    self.assemble = {}
                    self.assemble[queueName] = imageMsg
                    
                    continue
                elif self.assemble_index > ts + 1.0/10:
                    print("ignore {} for later".format(queueName))
                    continue
                else:
                    self.assemble[queueName] = imageMsg
                    break
        # check for full assembly

        if len(self.assemble) == self.size:
            
            self.latest = self.assemble

            self.assemble = {}

            # print("success assembling {}".format(self.assemble_index / 1.e9))

            self.assemble_index = -1

    def pop_latest(self):

        with self.lock:
            return self.latest

class RosbagDatasetRecorder:

    def __init__(self, image_topics, bag_name):

        self.image_sub = ApproximateSyncImageSubscriber(image_topics)

        self.bag = None
        self.bag_name = bag_name

        self.recording = False
        self.thread = None

        self.num_snapshot = 0

        self.last_imu_seq = -1

        self.bridge = CvBridge()

    def start(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def run(self):
        print("ready for recording, displaying cv2 preview...")

        if self.bag is None:
            self.bag = rosbag.Bag(self.bag_name, mode='w', compression=rosbag.Compression.NONE)
        
        while not rospy.is_shutdown():
            latest = self.image_sub.pop_latest()

            if latest is not None:
                for topic in latest:

                    imageMsg = latest[topic]

                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(imageMsg, desired_encoding="mono8")
                    except CvBridgeError as e:
                        print(e)

                    cv_image = cv2.resize(cv_image, None , fx=0.5, fy=0.5)
                    cv2.imshow(topic,cv_image)

                key = cv2.waitKey(100)

                if key == ord('q'):
                    self.stop_record()
                    cv2.destroyAllWindows()
                    return
                elif key == ord(' '):
                    self.start_record()
                    



    def start_record(self):
            
        # only capture images
        image_dict = self.image_sub.pop_latest()

        stamp = None

        for imageName in image_dict:
            imageMsg = image_dict[imageName]

            # force time alignment
            if stamp is None:
                stamp = imageMsg.header.stamp
            imageMsg.header.stamp = stamp
            # imgMsg, stamp = self.ecal2ros_image(imageMsg)
            self.bag.write(imageName, imageMsg, stamp)

        self.num_snapshot += 1

        print("snapshot taken {}".format(self.num_snapshot))

    def stop_record(self):

        if self.bag is not None:
            self.bag.close()
            print("bag file closes")


def main():

    image_topics = ["/tiscamera_ros/fisheye_left/image_rect_raw",
       "/tiscamera_ros/fisheye_right/image_rect_raw"]

    bag_name = "./rosbag/" + datetime.now().strftime("%Y-%m-%d-%I-%M-%S-") + "_recording.bag"

    rospy.init_node("calibration_dataset_recorder")
    
    recorder = RosbagDatasetRecorder(image_topics, bag_name)

    recorder.start()

    rospy.spin()


if __name__ == "__main__":
    main()