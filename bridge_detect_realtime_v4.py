#!/usr/bin/env python

import cv2 
import numpy as np
import imutils
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import rospy


bridge = CvBridge()
img_pub = rospy.Publisher("/center_point_img",Image)
bridge_pub = rospy.Publisher("/bridge_details",Point)


def differential_filter(img, window_size=7):

	if (window_size%2)==0:
		# print 'odd window size, adding 1'
		window_size=window_size+1
	if window_size==1:
		# print 'window size=1, making 3'
		window_size=3
	padding= (window_size-1)/2
	img_pad = cv2.copyMakeBorder(img, padding, padding, padding, padding, cv2.BORDER_REPLICATE)

	img_out=0*img.copy()
	for j in range(img.shape[1]):
		for i in range(img.shape[0]):
			window= img_pad[i:(i+window_size),j:(j+window_size)]
			img_out[i,j]= np.amax(window)-np.amin(window)

	return img_out

def find_bridge(image_raw):


	print '--------------'

	cannyval=int(np.amin(image_raw.shape)*.35)#.31)
	print cannyval

	adaptive_filtersize= int(np.amin(image_raw.shape)*.01)
	print adaptive_filtersize


	img_filt=differential_filter(image_raw,adaptive_filtersize)
	
	mask = cv2.inRange(img_filt, 0, 13)


	mask2 = cv2.inRange(image_raw, 100, 230)

	mask= cv2.bitwise_and(mask,mask2)

	# edges = cv2.Canny(mask,cannyval,cannyval*3,apertureSize = 3) 

	# cv2.imshow('canny',edges)

	cnts = cv2.findContours(mask, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	maxA=0
	countours_exist=False
	got_bridge=False
	cx0=0
	cy0=0
	bridge_angle=0
	first_hole_angle=None

	for c in cnts:
		
		# A = cv2.contourArea(c)
		A = cv2.arcLength(c,False)
		countours_exist=True
		if A>maxA:
			maxA=A
			max_contour=c

	if countours_exist:
			checked=np.array([[-1,-1]])
			

			p_min= .15* cv2.arcLength(max_contour,True)
			equi_radius = .125/.15*p_min #.5*np.sqrt(4*maxA/np.pi)
			M = cv2.moments(max_contour)
			cx0 = int(M['m10']/M['m00'])
			cy0 = int(M['m01']/M['m00'])

			cv2.drawContours(image_raw, [max_contour], -1, 255, 4)
			for c in cnts:
				perimeter = cv2.arcLength(c,True)
				if perimeter>p_min: #if its not very small
					M = cv2.moments(c)
					cx = int(M['m10']/(M['m00']+.0000000001))
					cy = int(M['m01']/(M['m00']+.0000000001))


					if np.equal(checked,[cx,cy]).all(1).any()==False:
						checked = np.append(checked, [[cx,cy]], axis=0)

						if np.linalg.norm(np.array([cx-cx0,cy-cy0]))< 1.2*equi_radius: #if its within the "bridge" body

							
							hull_area = cv2.contourArea(cv2.convexHull(c))
							solidity = float(M['m00'])/hull_area
							# print 'solidity: ',solidity
							if solidity>.8:


								rect=cv2.minAreaRect(c)
								# print '--'
								# print rect

								angle=rect[2]
								L=rect[1][0]
								W=rect[1][1]

								cv2.drawContours(image_raw, [c], -1, 0, 2)
								if L<W: #if length is shorter than width. then the angle is actually 90deg off
									W=rect[1][0]
									L=rect[1][1]
									if rect[2]<0:
										angle=rect[2]+90
									else:
										angle=rect[2]-90

								print angle, L/W
								if np.abs((L/W)-3)<.66: #if its got the aspect ratio of the hole
									# print 'got a rectangle'
									cv2.drawContours(image_raw, [c], -1, 125, 3)
									if first_hole_angle==None:
										first_hole_angle=angle
				 					else:
				 						if np.abs(angle-first_hole_angle) < 5:
				 							got_bridge=True

	if got_bridge or first_hole_angle is not None:

		if got_bridge==False:
			print 'NOT SUPER SURE'
		angle=(first_hole_angle+90) * 3.14/180

		x2 = int(round(cx0 + 1000* np.cos(angle)))
		y2 = int(round(cy0 + 1000 * np.sin(angle)))
		x1 = int(round(cx0 - 1000* np.cos(angle)))
		y1 = int(round(cy0 - 1000 * np.sin(angle) ))


		cv2.line(image_raw,(x1,y1),(x2,y2),(0,255,0),2)
		cv2.circle(image_raw,(int(cx0),int(cy0)),3,(255,0,0),-1)

		# cv2.imshow('image_raw',image_raw)
	

	# print("--- %s seconds ---" % (time.time() - start_time))
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	return got_bridge, image_raw, cx0, cy0, bridge_angle

def img_callback(data):
    img = bridge.imgmsg_to_cv2(data, "bgr8")

    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	interference_ratio= .14 #how far from the left side of the screen the lense shit is in frame
	
	x_start= int(interference_ratio*img.shape[1])
	img= img[:,x_start:]

	#scale image down for speed, reduces accuracy
	imgScale=.7

	newX,newY = img.shape[1]*imgScale, img.shape[0]*imgScale
	img = cv2.resize(img,(int(newX),int(newY)),cv2.INTER_AREA)

    gotit, img_center,x,y, angle = find_bridge(img)


    
    outt=Point()
    outt.x= int(x*(1/imgScale))
    outt.y= int(y*(1/imgScale))
    outt.z=angle
    img_pub.publish(bridge.cv2_to_imgmsg(img_center, "bgr8"))
    bridge_pub.publish(outt)

def find_bridge_main():
	rospy.init_node('find_bridge', anonymous=True)
    img_sub = rospy.Subscriber("/duo3d/left/image_rect", Image, img_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        find_bridge_main()
    except rospy.ROSInterruptException:
        pass
