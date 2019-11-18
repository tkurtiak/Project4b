# Train GMM
import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 


# Import Dataset
# Save/Load File name for RGB values
#filename = 'MaroonWindow_2.npy'
#GMMout = 'Maroon2_GMM_HSV_5'

#filename = 'TheWall.npy'
#GMMout = 'TheWallGMM_5'


filename = 'WoodenFeet.npy'
GMMout = 'WoodenFeetGMM_3'

# load current training dataset
BGR_set = np.load(filename)[0:3]
#HSV_set = np.load(filename)[3:]
data = BGR_set

# Set number of Gaussian models
k = 3



# Randomly select initial values for each model
mean = np.random.random_sample((k,3))*255
cov_rand = np.random.random_sample((k,3,3))*(3*255**(0.5))
cov = np.zeros(cov_rand.shape)
for i in range(0,k):
	cov[i] = np.dot(cov_rand[i],cov_rand[i].T)
pi = np.random.random_sample(k)
pmax = np.ones(k)

error = 100
a = np.zeros((data.shape[1],k))
meanprod = np.zeros((k,3))
meantotal = np.zeros(k)
covsum = np.zeros((k,3,3))
mean_new = np.zeros(mean.shape)
cov_new = np.zeros(cov.shape)
pi_new = np.zeros(pi.shape)
p = np.zeros((data.shape[1],k))
sumPip = 0
it = 0

# Iterate 
while error > .7:#.15:
	it = it+1
	icov = np.linalg.inv(cov)
	p0 = 1/(((2*3.14159)**3*np.linalg.det(cov)))**(0.5)
	# Run through each cluster, j
	mean_old = mean*1
	pi_old = pi*1
	# now run through each individual sample BGR, i
	for i in range(0,data.shape[1]):
		for j in range(0,k):
			pmax[j] =  p0[j]*np.exp(.5*np.linalg.multi_dot([[0,0,0],icov[j],[0,0,0]]))
			p[i,j] = p0[j]*np.exp(-.5*np.linalg.multi_dot([(data[:,i]-mean[j]),icov[j],(data[:,i]-mean[j])]))/pmax[j]
			sumPip = sumPip + pi[j]*p[i,j]
		a[i,:] = pi*p[i,:]/sumPip
		meanprod[:,:] = meanprod[:] + np.outer(a[i,:],data[:,i])
		meantotal[:] =  meantotal[:] + a[i,:] 
		for j in range(0,k):
			covsum[j,:,:] = covsum[j,:,:] + a[i,j]*np.outer(data[:,i]-mean[j],data[:,i]-mean[j])
		sumPip = 0  # Reset for next datapoint
	for j in range(0,k):
		#mean_new[j] = meanprod[j]/meantotal[j]
		mean[j] = meanprod[j]/meantotal[j]
		cov[j] = covsum[j]/meantotal[j]
		pi[j] = meantotal[j]/data.shape[1]

	#error = np.linalg.norm(np.dot(mean_old,pi_old)-np.dot(mean,pi))
	error = np.linalg.norm(mean_old-mean)
	#error = np.linalg.norm(pi_old-pi)
	print(error)


print(mean)
print(cov)
print(pi)


# Save GMM
np.savez(GMMout,k,mean,cov,pi)