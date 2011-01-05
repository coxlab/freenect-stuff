#!/usr/bin/env python

import sys

from pylab import *
from mpl_toolkits.mplot3d import Axes3D

from PIL import Image

import cv

# values from ros kinect_camera package
depthCamMatrix = matrix([[ 585.05108211, 0.00000000, 315.83800193], [0.00000000, 585.05108211, 242.94140713], [0.00000000, 0.00000000, 1.00000000 ]])
depthDistCoeffs = matrix([ 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000 ])
depthRectMatrix = matrix([[ 1., 0., 0.], [0., 1., 0.], [0., 0., 1. ]])
depthProjMatrix = matrix([[ 585.05108211, 0.00000000, 315.83800193, 0.], [0.00000000, 585.05108211, 242.94140713, 0.], [0.00000000, 0.00000000, 1.00000000, 0. ]])

rgbCamMatrix = matrix([[ 526.37013657, 0.00000000, 313.68782938], [0.00000000, 526.37013657, 259.01834898], [0.00000000, 0.00000000, 1.00000000 ]])
rgbDistCoeffs = matrix([ 0.18126525, -0.39866885, 0.00000000, 0.00000000, 0.00000000 ])
rgbRectMatrix = matrix([[ 1., 0., 0.], [0., 1., 0.], [0., 0., 1. ]])
rgbProjMatrix = matrix([[ 526.37013657, 0.00000000, 313.68782938, 0.], [0.00000000, 526.37013657, 259.01834898, 0.], [0.00000000, 0.00000000, 1.00000000, 0. ]])

shiftScale = 0.125
shiftOffset = 1088.6594#1084.0
baseline = 0.07219#0.075

Q = matrix([[1., 0., 0., -depthCamMatrix[0,2]],
            [0., 1., 0., -depthCamMatrix[1,2]],
            [0., 0., 0., depthCamMatrix[1,1]],
            [0., 0., 1./baseline, 0.]])

S = matrix([[0.999979, 0.006497, -0.000801, -0.025165],
            [-0.006498, 0.999978, -0.001054, 0.000047],
            [0.000794, 0.001059, 0.999999, -0.004077],
            [0., 0., 0., 1.]])

# S = matrix([[1., 0., 0., -0.25],
#             [0., 1., 0., 0.],
#             [0., 0., 1., 0.],
#             [0., 0., 0., 1.]])

P = matrix(zeros((3,4)))
P[:,:3] = rgbCamMatrix

depthToRgb = P * S * Q

fT = depthCamMatrix[0,0] * baseline # for point cloud generation

# print array(depthToRgb)
# sys.exit(1)

objDir = 'objs/'
if len(sys.argv) != 3:
    print "Usage: dat2obj.py <scanDir> <scanIndex>"
    print " Outputs to: %s" % objDir
    sys.exit(1)
# assert len(sys.argv) == 3

dataDir = sys.argv[1]
fI = int(sys.argv[2]) # frame index

rgbFile = open('%s/rgb_%i.dat' % (dataDir, fI), 'rb')
depthFile = open('%s/depth_%i.dat' % (dataDir, fI), 'rb')

im = Image.fromstring('RGB', (640,480), rgbFile.read())
# im.save('%s/%i.png' % (objDir, fI))

# rectify RGB image (and undistort)
cvIm = cv.CreateImageHeader(im.size, cv.IPL_DEPTH_8U, 3)
cv.SetData(cvIm, im.tostring())
cvRgbCamMatrix = cv.fromarray(array(rgbCamMatrix))
cvRgbDistCoeffs = cv.fromarray(array(rgbDistCoeffs))
cvDstIm = cv.CreateImage(cv.GetSize(cvIm), cvIm.depth, cvIm.channels)
cv.Undistort2(cvIm, cvDstIm, cvRgbCamMatrix, cvRgbDistCoeffs)
cv.SaveImage('%s/%i.png' % (objDir, fI), cvDstIm)
# print im.size, cv.GetSize(cvIm)
# print im.tostring() == cvIm.tostring()
# sys.exit(1)




depth = fromstring(depthFile.read(),dtype=uint16).reshape((480,640))

rgbFile.close()
depthFile.close()

# reconstruct depth
#depth = -325.616 / (depth - 1084.61)
depth = shiftScale * (shiftOffset - depth)
z = fT / depth

obj = open('%s/%i.obj' % (objDir, fI),'w')

genFaces = True

depthLow = 0.01
depthHigh = 10
depthDelta = 0.05

# mapping of point index in obj file to point index in raw data
objToRaw = []
# mapping of point index in raw file to point index in obj file
rawToObj = zeros(depth.shape,dtype=int) - 1

objPointCount = 0

for u in xrange(depth.shape[1]): # was x
    for v in xrange(depth.shape[0]): # was y
        rawIndex = u+v*depth.shape[1]
        if z[v,u] > depthLow and z[v,u] < depthHigh:
            # texture
            uvd1 = matrix([u, v, depth[v,u], 1.]).transpose()
            uvw = array((depthToRgb * uvd1).transpose())[0]
            tu = (uvw[0]/uvw[2] + 0.5) / float(depth.shape[1]-1)
            tv = 1.0 - (uvw[1]/uvw[2] + 0.5) / float(depth.shape[0]-1)
            # print tu, tv # TODO texture is offset by a substantial amount, is this mesh specific or due to lack of rectification?
            # u,v = x / float(depth.shape[1]-1), y / float(depth.shape[0]-1)
            obj.write('vt %.6f %.6f\n' % (tu,tv))
                        
            # location
            x = (u - depthCamMatrix[0,2]) / depthCamMatrix[0,0] * z[v,u]
            y = (v - depthCamMatrix[1,2]) / depthCamMatrix[1,1] * z[v,u]
            obj.write('v %.6f %.6f %.6f\n' % (x, y, z[v,u]))
            
            # store indices for meshing
            objIndex = len(objToRaw)
            objToRaw.append(rawIndex)
            rawToObj[v,u] = objIndex + 1
        
        if genFaces:
            if v > 0:
                if u > 0:
                    x0 = u-1
                    y0 = v-1
                    p0 = rawToObj[y0,x0]
                    #p0 = rawToObj[rawIndex-depth.shape[1]-1]
                    #y0,x0 = divmod(p0,depth.shape[1])
                    x1 = u
                    y1 = v-1
                    p1 = rawToObj[y1,x1]
                    #p1 = rawToObj[rawIndex-depth.shape[1]]
                    #y1,x1 = divmod(p1,depth.shape[1])
                    x2 = u
                    y2 = v
                    p2 = rawToObj[y2,x2]
                    #p2 = rawToObj[rawIndex]
                    #y2,x2 = divmod(p2,depth.shape[1])
                    x3 = u-1
                    y3 = v
                    p3 = rawToObj[y3,x3]
                    #p3 = rawToObj[rawIndex-1]
                    #y3,x3 = divmod(p3,depth.shape[1])
                    
                    d0 = z[y0,x0]
                    d1 = z[y1,x1]
                    d2 = z[y2,x2]
                    d3 = z[y3,x3]
                    
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