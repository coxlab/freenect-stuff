#!/usr/bin/env python

import sys

from pylab import *
from mpl_toolkits.mplot3d import Axes3D

outputDir = 'kinect_scans/4'
savePlot = True

assert len(sys.argv) == 3

dataDir = sys.argv[1]
fI = int(sys.argv[2]) # frame index

rgbFile = open('%s/rgb_%i.dat' % (dataDir, fI), 'rb')
depthFile = open('%s/depth_%i.dat' % (dataDir, fI), 'rb')

rgb = fromstring(rgbFile.read(),dtype=uint8).reshape((480,640,3))
# TODO, check if depth is a color code (3 bytes per pixel) or straight depth (2 bytes per pixel)
depth = fromstring(depthFile.read(),dtype=uint16).reshape((480,640))

# reconstruct depth
depth = -325.616 / (depth - 1084.61)
#
#   2048 means 11-bit depth
#   Conversion to meters (per ROS)
#   linearizationTable[depth] = -325.616F / ((GLfloat) depth + -1084.61F);
#
#for (i=0; i<2048; i++) {
#   float v = i/2048.0;
#   v = powf(v, 3)* 6;
#   t_gamma[i] = v*6*256;
# }
#   uint16_t t_gamma[2048];
#   pval = t_gamma[depth[i]]
#   lb = pval & 0xff
#       r       g       b
#   0   255     255-lb  255-lb  : r == 255, g == b
#   1   255     lb      0       : r == 255, g != b, b == 0 
#   2   255-lb  255     0       : g == 255, b == 0
#   3   0       255     lb      : r == 0, g == 255
#   4   0       255-lb  255     : r == 0, b == 255
#   5   0       0       255-lb  : r == 0, b == 0, r == b
#   def 0       0       0       : all 0
#
#   if r == 255 and g == b:
#       lb = 255 - g
#       hb = 0
#   elif r == 255 and b == 0:
#       lb = g
#       hb = 1
#   elif g == 255 and b == 0:
#       lb = 255 - r
#       hb = 2
#   elif r == 0 and g == 255
#       lb = b
#       hb = 3
#   elif r == 0 and b == 255:
#       lb = 255 - g
#       hb = 4
#   elif r == 0 and b == 0:
#       lb = 255 - b
#       hb = 5
#   else:
#       lb = 0
#       hb = 0

figure()
subplot(221)
imshow(rgb)
title('RGB')

subplot(222)
imshow(depth)
title('Depth')

subplot(223)
imshow((depth > 3) * depth)
title('Depth > 3')

subplot(224)
imshow((depth < 3) * depth)
title('Depth < 3')

# ax = Axes3D(figure())
# y, x = divmod(arange(640*480),640)
# z = depth.flatten()
# ax.scatter(x,y,z)
if savePlot:
    savefig('%s/%i.png' % (outputDir, fI))
else:
    show()
