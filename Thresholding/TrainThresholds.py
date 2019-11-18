import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 
#import color_correct as cc


## Selected training img
# imgname = 'Images/InitislSet.png'
# Image Select
import Tkinter, tkFileDialog

root = Tkinter.Tk()
root.withdraw()
imgname = tkFileDialog.askopenfilename()

# Save/Load File name for RGB values
#filename = 'MaroonWindow_2.npy'

#filename = 'WoodenFeet.npy'#'Final_middlevals_YellowWindow.npy'
filename = 'TheWall.npy'

# load image
img_raw = cv2.imread(imgname)
# Color correct
#img_corrected = cc.color_correct(img_raw)
img_corrected = img_raw
# SCALE DOWN IMAGE SIZE
scale = 1
width = int(img_corrected.shape[1] * scale)
height = int(img_corrected.shape[0] * scale)
dim = (width, height)
# resize image
img = cv2.resize(img_corrected, dim, interpolation = cv2.INTER_AREA)



blur = cv2.medianBlur(img,5)

gray = cv2.cvtColor(blur,cv2.COLOR_RGB2GRAY) 
mask = (gray>235).astype('uint8')*255
img_c_blur_sat = cv2.bitwise_not(blur,blur,mask = mask)
img_c_blur_sat_HSV = cv2.cvtColor(img_c_blur_sat, cv2.COLOR_BGR2HSV)

# load current training dataset
try:  # If it exists, load
	temp = np.load(filename)
	B = temp[0]
	G = temp[1]
	R = temp[2]
	H = temp[3]
	S = temp[4]
	V = temp[5]
except:	# Else initialize to zero
	print('no starting data found, creating my own. ')
	B =np.array([])
	G =np.array([])
	R =np.array([])
	H =np.array([])
	S =np.array([])
	V =np.array([])

# Select rectangular regions of interest.  
# Be careful not to select bad data or this will make our life hell
samples = cv2.selectROIs('Window',img_c_blur_sat)
cv2.destroyAllWindows()



for sample in samples:
	# sample has a starting point, top left corner of box, and a width, and a height
	yS = slice(sample[0],sample[0]+sample[2],1)
	xS = slice(sample[1],sample[1]+sample[3],1)
	# extract sample space
	sampleImg = img_c_blur_sat[xS,yS,:]
	sample_HSV_img = img_c_blur_sat_HSV[xS,yS,:]

	# Convert the RGB values to 1D arrays for processing

	B = np.append(B,np.ndarray.flatten(sampleImg[:,:,0],order='C'))
	G = np.append(G,np.ndarray.flatten(sampleImg[:,:,1],order='C'))
	R = np.append(R,np.ndarray.flatten(sampleImg[:,:,2],order='C'))

	H = np.append(H,np.ndarray.flatten(sample_HSV_img[:,:,0],order='C'))
	S = np.append(S,np.ndarray.flatten(sample_HSV_img[:,:,1],order='C'))
	V = np.append(V,np.ndarray.flatten(sample_HSV_img[:,:,2],order='C'))


# Save newly added training points to dataset
np.save(filename,np.vstack((B,G,R,H,S,V)))

plotData = 0

if plotData == 1:
	# Make a 3D ScaTTER Of RGB values
	plt.ion() # This prevents the program from hanging at the end
	fig = plt.figure()
	ax = Axes3D(fig)

	fig2 = plt.figure()
	ax2 = Axes3D(fig2)
	#ax = fig.add_subplot(111, projection='3d')
	i=0
	for i in range(0,R.shape[0]-1):
		ax.scatter(R[i], G[i], B[i],color='k')
		ax2.scatter(H[i], S[i], V[i],color='k')

	ax.set_xlabel('B')
	ax.set_ylabel('G')
	ax.set_zlabel('R')
	ax2.set_xlabel('H')
	ax2.set_ylabel('S')
	ax2.set_zlabel('V')
	plt.show()
