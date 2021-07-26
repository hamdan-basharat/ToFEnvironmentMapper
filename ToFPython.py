'''
2DX4 Final Project - Embedded Systems
Hamdan Basharat - basham1 - 400124515

Python program to visually represent data from ToF LIDAR system. Using distance mesaurements obtained from the ToF
a point cloud of data points is created to show room mapping and displacement of them motor/sensor unit. Note that
my ToF sensor broke during the later stages of testing so the measurements used are from a provided dataset. 

Last Updated: April 9th, 2020
'''
#Imported libraries
import serial
import csv
import numpy as np
import open3d as o3d
import math

#s = serial.Serial("COM7", 115200)
#print("Opening: " + s.name)

#Opening the files that contain or will store the data points
saveFile = open("tof_radar.xyz", "w+") #.XYZ file type
readFile1 = open("dataset_x0.txt", "r+")
readFile2 = open("dataset_x0.2.txt", "r+")
readFile3 = open("dataset_x0.4.txt", "r+")
readFile4 = open("dataset_x0.6.txt", "r+")
readFile5 = open("dataset_x0.8.txt", "r+")
saveFile.truncate(0) #clears the file contents from previous run

#Each loop pulls data from a rotation of the motor. Four displacements were used. 
for i in range(512): #The sensor took 512 measurements
    dist = readFile1.readline() #Reads the distance 
    y = float(dist)*math.sin(math.radians(i*0.703125)) #Converts distance to Y and Z components
    z = float(dist)*math.cos(math.radians(i*0.703125))
    point = str(y) + " " + str(z) + " 0\n" #Converts to string (displacement put on z because open3D changes colors for z-displacements)
    saveFile.write(point) #Writes data point to .xyz file
readFile1.close(); #Closes the data file that was being read from
for i in range(512):
    dist = readFile2.readline()
    y = float(dist)*math.sin(math.radians(i*0.703125))
    z = float(dist)*math.cos(math.radians(i*0.703125))
    point = str(y) + " " + str(z) + " -200\n" #Displacement of -0.2m (x-plane but shown in z-plane)
    saveFile.write(point)
readFile2.close();
for i in range(512):
    dist = readFile3.readline()
    y = float(dist)*math.sin(math.radians(i*0.703125))
    z = float(dist)*math.cos(math.radians(i*0.703125))
    point = str(y) + " " + str(z) + " -400\n"
    saveFile.write(point)
readFile3.close();
for i in range(512):
    dist = readFile4.readline()
    y = float(dist)*math.sin(math.radians(i*0.703125))
    z = float(dist)*math.cos(math.radians(i*0.703125))
    point = str(y) + " " + str(z) + " -600\n"
    saveFile.write(point)
readFile4.close();
for i in range(512):
    dist = readFile5.readline()
    y = float(dist)*math.sin(math.radians(i*0.703125))
    z = float(dist)*math.cos(math.radians(i*0.703125))
    point = str(y) + " " + str(z) + " -800\n"
    saveFile.write(point)
readFile5.close();

print("Closing: " + saveFile.name) #Closes the .xyz file that stores data points
saveFile.close();

#Code to convert Serial to XYZ file format (can't use anymore since sensor broke)
'''
for i in range(8):
    x = s.readline()    # read one line
    dist = x.decode()      # convert type to str
    dist = dist.rstrip()
    print(dist)
    y = float(dist)*math.sin(math.radians(i*45))
    z = float(dist)*math.cos(math.radians(i*45))
    point = str(y) + " " + str(z) + " 0\n"
    saveFile.write(point)
print("Closing: " + s.name)
s.close();
saveFile.close();
'''

print("Testing IO for point cloud ...")
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format='xyz')
print(pcd)

print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd]) #Creates a point cloud of the data points

pt1 = 0     #vertex labels
pt2 = 1
pt3 = 2
pt4 = 3
po = 0      #offset within plane

lines = []  #empty array for a line set

#connect all points within plane
for x in range(512):
    lines.append([pt1+po,pt2+po])
    lines.append([pt2+po,pt3+po])
    lines.append([pt3+po,pt4+po])
    lines.append([pt4+po,pt1+po])
    po += 5;

#reset variables
pt1 = 0
pt2 = 1
pt3 = 2
pt4 = 3
po = 0      #offset within plane
do = 4      #displacement offset

#connect vertices between planes
for x in range(511):
    lines.append([pt1+po,pt1+do+po])
    lines.append([pt2+po,pt2+do+po])
    lines.append([pt3+po,pt3+do+po])
    lines.append([pt4+po,pt4+do+po])
    po += 5;

#Constructor call to create line set
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
line_set.lines = o3d.utility.Vector2iVector(lines)

#3D visualization of LIDAR mapping
o3d.visualization.draw_geometries([line_set])  
