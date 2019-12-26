# -*- coding: utf-8 -*-

# code for SGBM
#first test is without point cloud implementation
import argparse
import sys
import os
#from PIL import Image
import numpy as np
from sklearn.preprocessing import normalize
import cv2
import matplotlib.pyplot as plt
import open3d as o3d
#import StereoVision
print('load')
left = cv2.imread('im0.png')
right = cv2.imread('im1.png')

window_size = 3;
#left_matcher = cv2.StereoSGBM_create(
#    minDisparity=0,
#    numDisparities=160,             
#    blockSize=5,
#    P1=8 * 3 * window_size ** 2,    
#    P2=32 * 3 * window_size ** 2,
#    disp12MaxDiff=1,
#    uniquenessRatio=15,
#    speckleWindowSize=0,
#    speckleRange=2,
#    preFilterCap=63,
#    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
#)

left_matcher = cv2.StereoSGBM_create(
    minDisparity=-39,
    numDisparities=144,             
    blockSize=5,
    P1=8 * 3 * window_size ** 2,    
    P2=32 * 3 * window_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

#WLS Filter

lmbda = 80000
sigma = 1.2
visual_multiplier = 1.0
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)
print('computing disparity...')
displ = left_matcher.compute(left, right)  
dispr = right_matcher.compute(right, left)  
displ = np.int16(displ)
dispr = np.int16(dispr)
filteredImg = wls_filter.filter(displ, left, None, dispr)
#cv2.imshow('Filtered Map', filteredImg)
filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
filteredImg = np.uint8(filteredImg)
imgplot=plt.imshow(filteredImg)
#plt.show()
#cv2.waitKey()
#cv2.destroyAlglWindows()

#rows, cols = filteredIm.shape
#c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
#valid = (filteredImg > 0) & (filteredImg < 255)
#z = np.where(valid, filteredImg / 256.0, np.nan)
#x = np.where(valid, z * (c - self.cx) / self.fx, 0)
#y = np.where(valid, z * (r - self.cy) / self.fy, 0)
#print "PCL"
#print np.dstack((x, y, z))
# 

#create point cloud with different files for the images
focalLength = 525.0
centerX = 319.5
centerY = 239.5
scalingFactor = 120.0
rgb = left
depth =filteredImg
#width, hrgbeight = depth.shape[:2]
#width1, height1 = filteredImg.shape[:2]
print ("-----")
#print width1,height1
#print width,height
print (rgb.shape)
print (depth.shape)
print ("-----")

#depth = depth.reshape((2000,2964))

#if rgb.size != depth.size:
#    raise Exception("Color and depth image do not have the same resolution.")
#if rgb.mode != "RGB":
#    raise Exception("Color image is not in RGB format")
#if depth.mode != "I":
#    raise Exception("Depth image is not in intensity format")


points = []    
#for v in range(rgb.size[1]):
#    for u in range(rgb.size[0]):
#        color = rgb.getpixel((u,v))
#        Z = depth.getpixel((u,v)) / scalingFactor
#        if Z==0: continue
#        X = (u - centerX) * Z / focalLength
#        Y = (v - centerY) * Z / focalLength
#        points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))

for v in range(rgb.shape[1]):
    for u in range(rgb.shape[0]):
        color = rgb[u,v]
        Z = depth[u,v] / scalingFactor
        if Z==0: continue
        X = (u - centerX) * Z / focalLength
        Y = (v - centerY) * Z / focalLength
        points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))
np.asarray(points)
np.asarray(color)
file = open('out.ply',"w")
file.write('''ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
end_header
%s
'''%(len(points),"".join(points)))
file.close()
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points) #numpy_points is your Nx3 cloud
pcd.colors = o3d.utility.Vector3dVector(color)
o3d.io.write_point_cloud("pointcloud.ply",pcd)
#pcd = o3d.io.read_point_cloud("out.ply")
#print(pcd)
#print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])































