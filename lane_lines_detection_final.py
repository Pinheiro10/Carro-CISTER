#!/usr/bin/env python
#-----------------------------------------------------------------------
#lane_lines_detection.py
#Deteccao de linhas na pista
#v00
#	Codigo alterado para repetir os dados da ultima linha detectada em caso de perda da linha
#v01
#	Adicionando dados de leitura da linha (X1, Y1, Slope)
#v02
#	Removing the display images (Look for cv2.imshow)
#	adding inital parameter (CAR)
#v03
#	Modificando deteccao de linhas (usando max e min de cada linha detectada)
#v04    --
#-----------------------------------------------------------------------
#Bibliotecas
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.signal import argrelextrema

from image_processing.msg import coords

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pickle
import glob
from numpy.linalg import inv
import os

car_name_TV = 0											#param list
publication_topic = 0									#topico a publicar dados do veiculo

DEBUG_VIDEO = 1

#--------------------------
#Topico a publicar
#pub=rospy.Publisher(publication_topic, coords, queue_size=10)
#pub=rospy.Publisher("/car1/"+"X_Y", coords, queue_size=10)
lines_old=[[0,0,0,0],[0,0,0,0]]

class image_converter:
	
	def __init__(self):
		self.image_pub = rospy.Publisher(car_name_TV+"camera/image_topic_2",Image,queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("zed/zed_node/right_raw/image_raw_color",Image,self.callback)

	def callback(self,data):
		global lines_old
		def select_rgb_white_yellow(image): 
			print
			# white color mask
			lower = np.uint8([158, 160, 160])
			upper = np.uint8([230, 230, 230])
			
			white_mask = cv2.inRange(image, lower, upper)
			# black color mask
			#lower = np.uint8([15, 15, 15])
			#upper = np.uint8([55, 55, 55])
			
			#lower = np.uint8([15, 15, 15])
			#upper = np.uint8([45, 45, 45])
			yellow_mask = cv2.inRange(image, lower, upper)
			# combine the mask
			mask = cv2.bitwise_or(white_mask, yellow_mask)
			#mask = cv2.bitwise_or(white_mask, white_mask)
			masked = cv2.bitwise_and(image, image, mask = mask)
			return masked
		def convert_hsv(image):
			return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		def convert_hls(image):
			return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
		def select_white_yellow(image):
			converted = convert_hsv(image)
			# white color mask
			#lower = np.uint8([ 70, 20, 85])
			#upper = np.uint8([80, 30, 100])
			#white_mask = cv2.inRange(converted, lower, upper)
			# black color mask
			
			lower = np.uint8([15, 15, 15])
			upper = np.uint8([47, 47, 47])
			
			
			#lower = np.uint8([0, 120, 0])
			#upper = np.uint8([170, 255, 130])
			yellow_mask = cv2.inRange(converted, lower, upper)
			# combine the mask
			mask = yellow_mask # cv2.bitwise_or(white_mask, yellow_mask)
			return cv2.bitwise_and(image, image, mask = mask)
		def convert_gray_scale(image):
			return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		def apply_smoothing(image, kernel_size=15):
			"""
			kernel_size must be postivie and odd
			"""
			return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
		def detect_edges(image, low_threshold=100, high_threshold=240): #50
			return cv2.Canny(image, low_threshold, high_threshold)
		def filter_region(image, vertices):
			"""
			Create the mask using the vertices and apply it to the input image
			"""
			mask = np.zeros_like(image)
			if len(mask.shape)==2:
				cv2.fillPoly(mask, vertices, 255)
			else:
				cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension		
			return cv2.bitwise_and(image, mask)	
		def select_region(image):
			"""
			It keeps the region surrounded by the `vertices` (i.e. polygon).	Other area is set to 0 (black).
			"""
			# first, define the polygon by vertices
			bottom_left	= [150, 720]
			top_left	 = [185, 480]
			bottom_right = [1050, 720]
			top_right	= [1025, 450] 
			# the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
			vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
			return filter_region(image, vertices)
		def hough_lines(image):
			"""
			`image` should be the output of a Canny transform.
			
			Returns hough lines (not the image with lines)
			"""
			return cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=50, minLineLength=20, maxLineGap=110)
		def draw_lines(image, lines, color=[255, 0, 0], thickness=2, make_copy=True):
			# the lines returned by cv2.HoughLinesP has the shape (-1, 1, 4)
			if make_copy:
				#print ("ok_2")
				image = np.copy(image) # don't want to modify the original
				#print ("ok_2.1")
			"""#print ("Lines: ",lines.shape)
			if ((lines) == None):			#number of lines
				msg = coords()
				msg.length = 999
				msg.slope = 999
				msg.intercept = 999
				msg.X2 = 999
				msg.X1 = 999
				msg.dif_X = 999
				msg.Y2 = 999
				msg.Y1 = 999
				msg.dif_Y = 999
				pub=rospy.Publisher(publication_topic, coords, queue_size=10)		
				pub.publish(msg)
				print ("Line[0]: ") 
			else:	"""
			for line in lines:
				print ("ok_3")
				for x1,y1,x2,y2 in line:
					cv2.line(image, (x1, y1), (x2, y2), color, thickness)
			return image
		
		
		
		def average_slope_intercept(lines):
			#O CONCEITO DE RIGHT E LEFT LANES NO EXISTE, SERIA UM PLACEHOLDER MAS O RETURN DESTA FUNcaO e INUTIL
			#APENAS CALCULA A INCLINAcAO DA LINHA QUE VAI PASSAR PARA O LINE DECISION!
			
			global publication_topic
			left_lines	= [] # (slope, intercept)
			left_weights	= [] # (length,)
			right_lines	= [] # (slope, intercept)
			right_weights = [] # (length,)
			print("---------------------------")
			#print ("Lines size: ", len(lines))
			#print ("Line: ")
			print (lines)
			#tam_lines = len(lines)	
			"""if ((lines) == None):			#number of lines
				msg = coords()
				msg.length = 999
				msg.slope = 999
				msg.intercept = 999
				msg.X2 = 999
				msg.X1 = 999
				msg.dif_X = 999
				msg.Y2 = 999
				msg.Y1 = 999
				msg.dif_Y = 999
				pub=rospy.Publisher(publication_topic, coords, queue_size=10)		
				pub.publish(msg)
				print ("Line[0]: ")
			else:"""	
			#print ("Line[2]: ", lines[2])
			tam_lines = len(lines)
			
			#print ("Array")
			#lines_array = np.array(lines)
			#print (lines_array)
			#print (lines_array[0,1])
			#modificado na versao 0.3 do codigo
			#i = 0

			#Este bloco deve sair entretanto. Este calcula a media da inclinacao somando todas as coordenadas
			#e depois divide pelo numero de linhas detetadas^
			#---------------------------------------------------------------------------------------------------------------
		#	sum_x1 = 0
		#	sum_x2 = 0
		#	sum_y1 = 0
		#	sum_y2 = 0
		#	
		#
		#	for line in lines:
		#		for x1, y1, x2, y2 in line:
		#			sum_x1 = sum_x1 + x1
		#			sum_x2 = sum_x2 + x2
		#			sum_y1 = sum_y1 + y1
		#			sum_y2 = sum_y2 + y2
		#			if x2==x1:
		#				continue # ignore a vertical line
		#			
		#			slope = (y2-y1)/(x2-x1)
		#			intercept = y1 - slope*x1
		#			length = np.sqrt((y2-y1)**2+(x2-x1)**2)
		#	x1 = sum_x1/tam_lines
		#	x2 = sum_x2/tam_lines
		#	y1 = sum_y1/tam_lines
		#	y2 = sum_y2/tam_lines
		#	if (x2<>x1):
		#		slope = (y2-y1)/(x2-x1)
		#	else:
		#		slope = 0
		#	intercept = y1 - slope*x1
		#	length = np.sqrt((y2-y1)**2+(x2-x1)**2)
			
			#--------------------------------------------------------------------------------------------------------------
			#Bloco experimental de novas inclinacoes--- building falta os x's e y's 
			slope_array=np.array([])
			len_array=np.array([])
			sum_x1=0
			sum_y1=0
			print("Before for line in lines")

			for line in lines:
				for x1,y1,x2,y2 in line:
					if x2==x1:        #maybe not needed
						continue
					slope_int=(y2-y1)/(x2-x1)
					np.append(slope_array,slope_int)
					np.append(len_array,np.sqrt((y2-y1)**2+(x2-x1)**2))
					sum_x1=sum_x1+x1
					sum_y1=sum_y1+y1
			
			slope= (np.sum(slope_int))/tam_lines
			length=(np.sum(len_array))/tam_lines
			x1=sum_x1/tam_lines
			y1=sum_y1/tam_lines
			angle=np.arctan(slope)
			y2=(np.sin(angle)*length)+y1
			x2=(np.cos(angle)*length)+x1
		
		#	slope=1
		#	length=1
		#	x1=1
		#	y1=1
		#	angle=1
		#	y2=2
		#	x2=2
	#
	#	
	#	
			intercept=1 #nao utilizado
			




			#For PID calculations ( steering )
			msg = coords()
			msg.length = length
			msg.slope = slope
			msg.intercept = intercept
			msg.X2 = x2
			msg.X1 = x1
			msg.dif_X = x2 - x1
			msg.Y2 = y2
			msg.Y1 = y1
			msg.dif_Y = y2 - y1
			pub=rospy.Publisher(publication_topic, coords, queue_size=10)		
			pub.publish(msg)
	
			# add more weight to longer lines ---- estas linhas nao tem qualquer significado, retornam NONE
 	
		#	left_lane	= np.dot(left_weights,	left_lines) /np.sum(left_weights)	if len(left_weights) >0 else None
		#	right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
			#print("left_lane", left_lane)
			#print("right_lane", right_lane)
			#else:
				#msg = coords()
				#msg.length = 999
				#msg.slope = 999
				#msg.intercept = 999
				#msg.X2 = 999
				#msg.X1 = 999
				#msg.dif_X = 999
				#msg.Y2 = 999
				#msg.Y1 = 999
				#msg.dif_Y = 999
				#pub=rospy.Publisher(publication_topic, coords, queue_size=10)		
				#pub.publish(msg)
			return 1   # left_lane, right_lane  (slope, intercept), (slope, intercept)
		
		
		def four_point_transform(image):
			# obtain a consistent order of the points and unpack them
			# individually
			rect = np.float32([(10, 370), (790, 370), (800, 500), (0, 500)] )
			(tl, tr, br, bl) = rect

			# compute the width of the new image, which will be the
			# maximum distance between bottom-right and bottom-left
			# x-coordiates or the top-right and top-left x-coordinates
			widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
			widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
			maxWidth = max(int(widthA), int(widthB))
			# compute the height of the new image, which will be the
			# maximum distance between the top-right and bottom-right
			# y-coordinates or the top-left and bottom-left y-coordinates
			heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
			heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
			maxHeight = max(int(heightA), int(heightB))

			# now that we have the dimensions of the new image, construct
			# the set of destination points to obtain a "birds eye view",
			# (i.e. top-down view) of the image, again specifying points
			# in the top-left, top-right, bottom-right, and bottom-left
			# order
			dst = np.array([
				[0, 0],
				[maxWidth - 1, 0],
				[maxWidth - 1, maxHeight - 1],
				[0, maxHeight - 1]], dtype = "float32")

			# compute the perspective transform matrix and then apply it
			M = cv2.getPerspectiveTransform(rect, dst)
			warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

			# return the warped image
			return warped, M
		######
	
		print("Chegamos aqui")
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		#print ("Image.step: ",data.step)
		#(rows,cols,channels) = cv_image.shape
		#if cols > 60 and rows > 60 :
		#	cv2.circle(cv_image, (50,50), 10, 255)

		masked = select_rgb_white_yellow(cv_image)
		chsv = convert_hsv(cv_image)
		#chls = convert_hls(cv_image)
		s_white_yellow = select_rgb_white_yellow(cv_image)
		gscale = convert_gray_scale(s_white_yellow)
		#a_smooth_5 = apply_smoothing(gscale, 5) #3, 5, 9, 11, 15, 17 (positive and odd)
		edges = detect_edges(gscale) #a_smooth_5
		#print ("Edges: ", edges)
		roi = select_region(edges)
		lines = hough_lines(roi)
		
		if (lines is not None):
			lines_old = lines
		else: 
			lines = lines_old
			#print (lines)
		#print("Lines len por pontos: ", len(lines))

		#hough_lines_mask(roi)


		#for line in lines:
		#	line.X1	
		
		#lines_colored = draw_lines(cv_image, lines)
		print("Before average_slope_intercept")
		average_slope_intercept(lines)
		lines_colored = draw_lines(cv_image, lines)
		perspective_transformed, M = four_point_transform(roi)

		
		#cv2.imshow("masked", masked)
		#cv2.imshow("chsv", chsv)
		#cv2.imshow("chls", chls)
		#cv2.imshow("s_white_yellow", s_white_yellow)
		#cv2.imshow("convert_gscale", gscale)
		#cv2.imshow("a_smooth_5", a_smooth_5)
		#cv2.imshow("edges", edges)
		#cv2.imshow("roi", roi)
		
		#Necessary images to debug the system!
		if DEBUG_VIDEO:
			cv2.imshow("cv_image", cv_image)
			cv2.imshow("masked", masked)
			cv2.imshow("lines_colored", lines_colored)
			cv2.imshow("perspective_transformed", perspective_transformed)


		#cv2.imwrite('04.png',cv_image)
		cv2.waitKey(1)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

#def main(args):


if __name__ == '__main__':
	param = sys.argv[1:]
	if len(param) != 0: 
		car_name_TV = "/" + param[0] +"/"
		#car_name_TV = "/" + "car1" +"/"
		print (car_name_TV)
		publication_topic = "/" + param[0] + "/X_Y"
	else:
		car_name_TV = "/" + "car1" +"/"
		print (car_name_TV)
		publication_topic = "/" + "car1" + "/X_Y"
		
	ic = image_converter()
	print ("Lane Lines Detection")
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
