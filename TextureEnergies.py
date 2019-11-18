#!/usr/bin/env python

import cv2 
import numpy as np
# from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
from scipy.signal import convolve2d


def texture_energy(img,X5,Y5):
	# masked= cv2.filter2D(img,-1,np.matmul(X5.T,Y5))
	masked = convolve2d(img,np.matmul(X5.T,Y5), mode='same')

	#using a window size of 15, summ the absolute value of the window
	masked = np.abs(masked)
	kernel= np.ones((15,15))
	# masked=cv2.filter2D(masked,-1,kernel)
	masked= convolve2d(masked,kernel, mode='same')
	return masked

def process_img(image_gray,cluster=True, window_size=13,corner_def=.8,num_points=1500):
	# image_gray = cv2.cvtColor(image_raw,cv2.COLOR_BGR2GRAY)

	#PASS THIS FUNCTION AN NxMX1 image!

	OGshape= image_gray.shape

	#TUNING PARAMS
	#preprocessing window size (must be odd)
	#window_size=13 #must be odd
	#hard to tell whats best here

	#corner definition (affects how many textures are classified)
	#is a ratio, 
	#larger number is "more accurate" -> gives more textures
	#smaller number gives less textures
	#.75-.85 is a good range
	#corner_def=.8

	#how many points to generate model off of
	#doesnt affect much, smaller is faster performance, leaving it is probably okay
	#num_points=1300.


	#using Laws' Texture Energy Masks

	#literature suggests we subtract a windowed average from each pixel
	avg_kernel = np.ones((window_size,window_size),np.float32)/(window_size**2)
	avgs = convolve2d(image_gray,avg_kernel, mode='same') #fuck opencv functions here we want any number is possible
	preprocess= image_gray-avgs

	#here are the fundamentals for the kernels
	L5=np.array([[1.,4.,6.,4.,1.]])
	E5=np.array([[-1.,-2.,0.,2.,1.]])
	S5=np.array([[-1.,0.,2.,0.,-1.]])
	R5=np.array([[1.,-4.,6.,-4.,1.]])

	#we compute the 9 different types of energies
	L5E5= texture_energy(preprocess,L5,E5)
	E5L5= texture_energy(preprocess,E5,L5)
	energy1= cv2.addWeighted(L5E5,0.5,E5L5,0.5,0)

	L5R5= texture_energy(preprocess,L5,R5)
	R5L5= texture_energy(preprocess,R5,L5)
	energy2= cv2.addWeighted(L5R5,0.5,R5L5,0.5,0)

	S5E5= texture_energy(preprocess,S5,E5)
	E5S5= texture_energy(preprocess,E5,S5)
	energy3= cv2.addWeighted(S5E5,0.5,E5S5,0.5,0)

	energy4= texture_energy(preprocess,S5,S5)

	energy5= texture_energy(preprocess,R5,R5)

	L5S5= texture_energy(preprocess,L5,S5)
	S5L5= texture_energy(preprocess,S5,L5)
	energy6= cv2.addWeighted(L5S5,0.5,S5L5,0.5,0)

	energy7= texture_energy(preprocess,E5,E5)

	E5R5= texture_energy(preprocess,E5,R5)
	R5E5= texture_energy(preprocess,R5,E5)
	energy8= cv2.addWeighted(E5R5,0.5,R5E5,0.5,0)

	S5R5= texture_energy(preprocess,S5,R5)
	R5S5= texture_energy(preprocess,R5,S5)
	energy9= cv2.addWeighted(S5R5,0.5,R5S5,0.5,0)

	#we want nx9 array where n is num pixels and cols are energies for clustering
	
	energies = np.stack((energy1,energy2,energy3,energy4,energy5,energy6,energy7,energy8,energy9), axis=2)
	textures = 0*image_gray
	best_n=0

	if cluster==True:
		Bigarray = np.concatenate((np.array([energy1.flatten()]).T,np.array([energy2.flatten()]).T,np.array([energy3.flatten()]).T,np.array([energy4.flatten()]).T,np.array([energy5.flatten()]).T,np.array([energy6.flatten()]).T,np.array([energy7.flatten()]).T,np.array([energy8.flatten()]).T,np.array([energy9.flatten()]).T), axis=1)
		wcss = []

		#subsample for this not to take ages
		indicies=np.arange(0,Bigarray.shape[0],int(Bigarray.shape[0]/num_points))
		last_inertia=0.
		best_n=0
		for i in range(1, 11):
			kmeans = KMeans(n_clusters=i, init='k-means++', max_iter=300, n_init=10)
			kmeans.fit(Bigarray[indicies,:])
			wcss.append(kmeans.inertia_)
			if last_inertia==0:
				last_inertia=kmeans.inertia_
			else:
				#choose n such that you've turned the "elbow"
				ratio= kmeans.inertia_/last_inertia
				last_inertia=kmeans.inertia_
				if ratio>corner_def and best_n==0:
					best_n=i-1
		if best_n==0:
			best_n=11


		# plt.plot(range(1, 11), wcss)
		# plt.title('Elbow Method')
		# plt.xlabel('Number of clusters')
		# plt.ylabel('WCSS')
		# plt.show()

		#label all the points with best n
		kmeans = KMeans(n_clusters=best_n, init='k-means++', max_iter=300, n_init=10)
		kmeans.fit(Bigarray[indicies,:])
		labels=kmeans.predict(Bigarray)
		
		
		textures= np.reshape(labels, OGshape)
		textures= textures*int(255./(best_n-1))

		# plt.imshow(textures)
		# plt.show()

		# plt.imshow(preprocess)
		# plt.show()


	return energies,textures, best_n




