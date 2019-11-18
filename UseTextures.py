#!/usr/bin/env python

import cv2 
import numpy as np
from TextureEnergies import process_img
from matplotlib import pyplot as plt

# image_raw = cv2.imread("images/river-ladder-real.jpg")
image_raw = cv2.imread("images/bag1/frame0016.jpg")

image_test = cv2.cvtColor(image_raw,cv2.COLOR_BGR2GRAY)

energies,textures,n = process_img(image_test)

print n

# plt.imshow(textures)
# plt.show()

samples = cv2.selectROIs('Window',image_raw)
cv2.destroyAllWindows()

for sample in samples:
	# sample has a starting point, top left corner of box, and a width, and a height
	yS = slice(sample[0],sample[0]+sample[2],1)
	xS = slice(sample[1],sample[1]+sample[3],1)
	# extract sample space
	samples = energies[xS,yS,:]
	sample_mean = np.mean(samples,axis = (0,1))
	sample_mean = sample_mean/(sample_mean[0])

	print sample_mean


#Some normalized wall samples:

#close

# [ 1.          0.38080184  0.06312699  0.0257981   0.11158824  0.33882957
#   0.17513777  0.08390845  0.04488437]

# [ 1.          0.39272319  0.07226188  0.03192738  0.1429813   0.36893064
#   0.18348883  0.08633428  0.05279082]

# [ 1.          0.35911775  0.05479997  0.02406273  0.11148474  0.33797429
#   0.14469462  0.07537624  0.04356881]

# [ 1.          0.37258428  0.06211781  0.02846046  0.12687118  0.36119608
#   0.1486398   0.08331642  0.04941522]

# [ 1.          0.3313263   0.0500558   0.02161037  0.10479043  0.35876854
#   0.1314492   0.06715502  0.0406697 ]


#far

# [ 1.          0.5368818   0.09749786  0.05001485  0.16231714  0.40065399
#   0.21605739  0.13035047  0.0766994 ]

# [ 1.          0.53047717  0.10279069  0.05124909  0.17326454  0.42310572
#   0.22005795  0.13858056  0.08169291]

# [ 1.          0.51824275  0.10333166  0.05312447  0.18922646  0.44833674
#   0.2243981   0.13813693  0.0865051 ]

# [ 1.          0.88633061  0.09491688  0.04868372  0.17619846  0.44697142
#   0.19450868  0.1304191   0.07628326]

# [ 1.          0.64070697  0.09069946  0.04507733  0.19472789  0.45587633
#   0.18454032  0.15945734  0.09122295]

# [ 1.          0.55688364  0.07401672  0.03419662  0.16205325  0.41357558
#   0.16744158  0.11239867  0.06570285]





