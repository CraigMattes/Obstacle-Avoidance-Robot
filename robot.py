from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from rplidar import RPLidarException
from roboviz import MapVisualizer
import numpy as np
import threading
from threading import Thread
import time
from breezycreate2 import *
from math import pi,cos,sin,tan,atan
np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
MIN_SAMPLES = 175

# Initialize an empty trajectory
trajectory = []

# We will use these to store previous scan in case current scan is inadequate
previous_distances = None
previous_angles    = None

class TotalRobot:
	def __init__(self):
		self.robot = Robot('COM3')
		self.lidar = Lidar('COM4')
		self.container = np.zeros(360)
		self.angle_0 = 30000
		self.angle_180 = 30000 # 30 meters is enough to be the threshold
		self.angle_90 = 30000
		self.angle_270 = 30000
		self.angle_360 = 30000
		self.right_container = np.empty(80) # 95-175
		self.left_container = np.empty(80) #185-265
		self.right_container.fill(30000) # The maximum distance that the lidar can detect is 3 meters
		self.left_container.fill(30000)  # Fill distance array with the maximum distance
		self.iterator = self.lidar.iter_scans()
		# Create an RMHC SLAM object with a laser model and optional robot model
		self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
		# Set up a SLAM display
		self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

		# Initialize empty map
		self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)


	def constructmap(self):
		time.sleep(.01)
		while True:
			# Extract (quality, angle, distance) triples from current scan
			items = [item for item in next(self.iterator)]
			# Extract distances and angles from triples
			distances = [item[2] for item in items]
			angles = [item[1] for item in items]

			for quality, angle, distance in items:
				if(quality != 0 and distance >= 150):
					#print(angle, distance, quality)
					if(angle > 185 and angle <= 265):
						self.left_container = distance
					elif(angle >= 95 and angle < 175):
						self.right_container = distance
					elif(angle >= 0 and angle <= 45):
						#print(angle, distance)
						self.angle_0 = distance
					elif(angle >= 90 and angle <= 95):
						#print(angle, distance)
						self.angle_90 = distance
					elif(angle >= 175 and angle <= 185):
						#print(angle, distance)
						self.angle_180 = distance
					elif(angle >= 265 and angle <= 270):
						#print(angle, distance)
						self.angle_270 = distance
					elif(angle >= 315 and angle < 360):
						#print(angle, distance)
						self.angle_360 = distance

			left_angle = np.amax(self.left_container)
			right_angle = np.amax(self.right_container)
			if(self.angle_0 < 625 and self.angle_360 < 625):
				if(right_angle > left_angle):
					self.robot.setTurnSpeed(100)
				else:
					self.robot.setTurnSpeed(-100)
			elif(self.angle_0 < 625 and self.angle_90 < 625 and self.angle_180 < 625):
				self.robot.setTurnSpeed(-100)
			elif(self.angle_270 < 625 and self.angle_360 < 625):
				self.robot.setTurnSpeed(100)
			else:
				self.robot.setForwardSpeed(175)

			if len(distances) > MIN_SAMPLES:
				self.slam.update(distances, scan_angles_degrees=angles)
				previous_distances = distances.copy()
				previous_angles = angles.copy()

			# If not adequate, use previous
			elif previous_distances is not None:
				self.slam.update(previous_distances, scan_angles_degrees=previous_angles)

			x, y, theta = self.slam.getpos()
			#trajectory.append((x,y))

			self.slam.getmap(self.mapbytes)
			#self.slam._getNewPosition(trajectory)
			#print(x)
			if not self.viz.display(x/1000.,y/1000., theta, self.mapbytes):
				self.lidar.stop_motor()
				self.lidar.disconnect()
				self.robot.setTurnSpeed(0)
				self.robot.close()
				break
				exit(0)


if __name__ == "__main__":
	robot = TotalRobot()
	robot.constructmap()
