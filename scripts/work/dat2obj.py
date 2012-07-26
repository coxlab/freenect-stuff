#!/usr/bin/env python

import sys

from pylab import *
from mpl_toolkits.mplot3d import Axes3D

from PIL import Image

assert len(sys.argv) == 3


objDir = 'objs/'
dataDir = sys.argv[1]
fI = int(sys.argv[2]) # frame index

rgbFile = open('%s/rgb_%i.dat' % (dataDir, fI), 'rb')
depthFile = open('%s/depth_%i.dat' % (dataDir, fI), 'rb')

#rgb = fromstring(rgbFile.read(),dtype=uint8).reshape((480,640,3))
im = Image.fromstring('RGB', (640,480), rgbFile.read())
im.save('%s/%i.png' % (objDir, fI))
# TODO, check if depth is a color code (3 bytes per pixel) or straight depth (2 bytes per pixel)
depth = fromstring(depthFile.read(),dtype=uint16).reshape((480,640))

rgbFile.close()
depthFile.close()

# reconstruct depth
depth = -325.616 / (depth - 1084.61)

obj = open('%s/%i.obj' % (objDir, fI),'w')

genFaces = True

depthLow = 1
depthHigh = 10
depthDelta = 1.

# mapping of point index in obj file to point index in raw data
objToRaw = []
# mapping of point index in raw file to point index in obj file
rawToObj = zeros(depth.shape,dtype=int) - 1

objPointCount = 0

for x in xrange(depth.shape[1]):
    for y in xrange(depth.shape[0]):
        rawIndex = x+y*depth.shape[1]
        if depth[y,x] > depthLow and depth[y,x] < depthHigh:
            u,v = x / float(depth.shape[1]-1), y / float(depth.shape[0]-1)
            obj.write('vt %.6f %.6f\n' % (u,v))
            obj.write('v %.6f %.6f %.6f\n' % ((u-0.5)*10., (v-0.5)*10., depth[y,x]))
            
            # store indices for meshing
            #rawIndex = x+y*depth.shape[1]
            objIndex = len(objToRaw)
            objToRaw.append(rawIndex)
            rawToObj[y,x] = objIndex + 1
        
        
        if genFaces:
            if y > 0:
                if x > 0:
                    x0 = x-1
                    y0 = y-1
                    p0 = rawToObj[y0,x0]
                    #p0 = rawToObj[rawIndex-depth.shape[1]-1]
                    #y0,x0 = divmod(p0,depth.shape[1])
                    x1 = x
                    y1 = y-1
                    p1 = rawToObj[y1,x1]
                    #p1 = rawToObj[rawIndex-depth.shape[1]]
                    #y1,x1 = divmod(p1,depth.shape[1])
                    x2 = x
                    y2 = y
                    p2 = rawToObj[y2,x2]
                    #p2 = rawToObj[rawIndex]
                    #y2,x2 = divmod(p2,depth.shape[1])
                    x3 = x-1
                    y3 = y
                    p3 = rawToObj[y3,x3]
                    #p3 = rawToObj[rawIndex-1]
                    #y3,x3 = divmod(p3,depth.shape[1])
                    
                    d0 = depth[y0,x0]
                    d1 = depth[y1,x1]
                    d2 = depth[y2,x2]
                    d3 = depth[y3,x3]
                    
                    if p1 != -1 and p3 != -1:
                        if abs(d1 - d3) < depthDelta:
                            if p0 != -1:
                                if abs(d1 - d0) < depthDelta:
                                    if abs(d3 - d0) < depthDelta:
                                        # print "Making face with:"
                                        # print '%.2f, %.2f, %.2f' % (d0-d1, d0-d3, d1-d3)
                                        obj.write('f %i/%i %i/%i %i/%i\n' % (p0,p0,p1,p1,p3,p3))
                            if p2 != -1:
                                if abs(d2 - d1) < depthDelta:
                                    if abs(d2 - d3) < depthDelta:
                                        # print "Making face with:"
                                        # print '%.2f, %.2f, %.2f' % (d1-d2, d1-d3, d2-d3)
                                        obj.write('f %i/%i %i/%i %i/%i\n' % (p1,p2,p2,p2,p3,p3))
obj.close()