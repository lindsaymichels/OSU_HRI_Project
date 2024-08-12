#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped

class Cluster:
    def __init__(self, pixel_size=0.01, dilation_size=None):
        """
        Initialize the Cluster class with specified pixel size and dilation size.
        """
        self.pixel_size = pixel_size
        self.dilation_size = dilation_size
        self.data = []

    def add_data(self, point):
        """
        Add a new data point to the cluster.
        """
        self.data.append(point)

    def fit(self):
        """
        Process the data points in the cluster, normalize, adjust, and apply image processing techniques.
        """
        if not self.data:
            print("No data to process")
            return

        # Normalize the data based on pixel size
        new_data = np.array([(int(x / self.pixel_size), int(y / self.pixel_size)) for x, y in self.data])
        print(f"Normalized data: {new_data}")

        # Adjust the data to start from (0,0)
        min_x, min_y = np.min(new_data[:, 0]), np.min(new_data[:, 1])
        new_data[:, 0] -= min_x
        new_data[:, 1] -= min_y
        print(f"Adjusted data: {new_data}")

        # Determine the dimensions of the image
        max_x, max_y = np.max(new_data[:, 0]), np.max(new_data[:, 1])
        print(f"Image Dimensions: ({max_x + 1}, {max_y + 1})")

        # Create an empty image and plot the points
        image = np.zeros((max_x + 1, max_y + 1), dtype=np.uint8)
        for x, y in new_data:
            image[x, y] = 255

        print(f"Image data:\n{image}")

        # Save the initial image
        image_path = '/home/prg/initial_image.png'
        plt.imsave(image_path, image, cmap='gray')
        print(f"Saved initial image to {image_path}")

        # Apply dilation and erosion if dilation_size is specified
        if self.dilation_size:
            kernel = np.ones((self.dilation_size, self.dilation_size), np.uint8)
            image = cv.dilate(image, kernel, iterations=1)
            image = cv.erode(image, kernel, iterations=1)

        # Save the dilated/eroded image
        image_path = '/home/prg/dilated_eroded_image.png'
        plt.imsave(image_path, image, cmap='gray')
        print(f"Saved dilated/eroded image to {image_path}")

        # Use connected components to label the image regions
        n_labels, labels, stats, centroids = cv.connectedComponentsWithStats(image, 8, cv.CV_8U)
        print(f'{n_labels} total regions')
        print(f'Stats: {stats}')

        # Count the number of objects based on the size of the regions
        num_objects = sum(np.logical_and(stats[:, 4] > 10, stats[:, 4] < 1000))
        print(f'{num_objects} actual objects')

        # Save the labeled image
        labeled_image_path = '/home/prg/labeled_image.png'
        plt.imsave(labeled_image_path, labels, cmap='nipy_spectral')
        print(f"Saved labeled image to {labeled_image_path}")

'''
def load_data():
    data = []
    try:
        with open('coordinates.txt', 'r') as f:
            for line in f:
                x, y = line.split()
                data.append((float(x), float(y)))
    except FileNotFoundError:
        rospy.logerr("Error: 'coordinates.txt' not found")
    except ValueError:
        rospy.logerr("Error: Incorrect data format in 'coordinates.txt'")
    
    return np.array(data)\
'''

class CoordinateClusterNode:
    def __init__(self):
        """
        Initialize the CoordinateClusterNode class, set up the ROS node, subscriber, and timer.
        """
        rospy.init_node('coordinate_cluster_node')
        self.sub = rospy.Subscriber('/human_coordinates', PointStamped, self.callback)
        self.cluster = Cluster(pixel_size=0.01, dilation_size=10)
        self.coordinates = []
        rospy.loginfo("CoordinateClusterNode initialized")

    def callback(self, msg):
        """
        Callback function for the subscriber to process incoming messages.
        """
        x = msg.point.x
        y = msg.point.y
        rospy.loginfo(f"Received coordinates: x={x}, y={y}")
        # Add received coordinates to the cluster
        self.cluster.add_data((x, y))
        rospy.loginfo("Data added to cluster")
        # Process the data (for example, fit clusters and filter)
        self.cluster.fit()
        rospy.loginfo("Cluster fitting completed")

    def process_data(self):
        """
        Process the collected data points by adding them to the cluster and fitting the cluster.
        """
        # Add all collected coordinates to the cluster and fit it
        for coord in self.coordinates:
            self.cluster.add_data(coord)
        self.cluster.fit()

        # Clear the coordinates list
        self.coordinates.clear()


        
    
    

if __name__ == '__main__':
    node = CoordinateClusterNode()
    rospy.spin()
