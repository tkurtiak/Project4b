import numpy as np
import cv2
## Selected training img
# imgname = 'Images/InitislSet.png'
# Image Select
import Tkinter, tkFileDialog

root = Tkinter.Tk()
root.withdraw()
imgname = tkFileDialog.askopenfilename()
img = cv2.imread(imgname)
#GMMin = 'Final_GoodData_GMM_BGR_3.npz'
GMMin = 'TheWallGMM_5.npz'
thresh = .08#1e-4#1e-5

#GMMin = 'WoodenFeetGMM_3.npz'
#thresh = .15#1e-4#1e-5


npzfile = np.load(GMMin)
k = npzfile['arr_0']
mean = npzfile['arr_1']
cov = npzfile['arr_2']
pi = npzfile['arr_3']
GMMargs = (thresh,k,mean,cov,pi)





def GMM(img,thresh,k,mean,cov,pi):
	#img_thresh = np.zeros(img.shape) # initialize black image
	print('Running GMM')
	icov = np.linalg.inv(cov)
	#p0 = 1/(((2*3.14159)**3*np.linalg.det(cov)))**(0.5)
	#pmax = np.zeros(k)
	#for j in range(0,k):
	#	pmax[j] =  p0[j]*np.exp(-.5*np.linalg.multi_dot([[0,0,0],icov[j],[0,0,0]]))

	# Loop through pixels and compare each value to the threshold.
	#x=0
	#y=0
	
	#po =  np.zeros(img.shape[:2])
	pmax = np.sum(pi[:])
	# for col in img:
	# 	for pixel in col:
	# 		p = np.zeros(k)
	# 		for j in range(0,k):
	# 			if (pixel == np.zeros(3)).all():
	# 				p[j] = 0
	# 			else:
	# 				temp = pixel-mean[j]
	# 				p[j] = pi[j]*np.exp(-.5*np.linalg.multi_dot([temp,icov[j],temp]))
	# 		po[y,x] = np.sum(p)
	# 		x = x+1
	# 	x = 0
	# 	y = y+1
	p = np.zeros((k,img.shape[0],img.shape[1]))
	img_flat= np.reshape(img, (img.shape[0]*img.shape[1] , 3))
	indx= np.where(img_flat[:,0] > 1)
	for i in range(1,k):
		diff= np.matrix(img_flat-mean[i])
		p_flat=np.zeros((diff.shape[0],1))
		p_flat[indx]= pi[i]*np.exp( -.5* np.sum( np.multiply( diff[indx] * icov[i], diff[indx]),axis=1))
		p[i,:,:]= np.reshape(p_flat, (img.shape[0],img.shape[1]))

	p_sum = np.sum(p,axis = 0)/pmax
	mask = cv2.inRange(p_sum,thresh,1)


	#pnorm = po/pmax#(po/np.max(po))

	#mask = (pnorm>=thresh).astype('uint8')*255
	#res = cv2.bitwise_and(img_thresh,img_thresh,mask = mask)
	return mask




outmask = GMM(img,thresh,k,mean,cov,pi)
cv2.imshow('Raw',img)
cv2.imshow('mask',outmask)
cv2.waitKey()
cv2.destroyAllWindows()