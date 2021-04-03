# -*- coding: UTF-8 -*-
import numpy as np
import open3d as o3d
import os
import glob
import time

A_SEMIMAJOR_AXIS = 6378137.0

F_FLATTENING = 1.0 / 298.257223563

B_SEMIMINOR_AXIS = 6356752.3142

# second eccentricity squared
ESQ_DASH = (A_SEMIMAJOR_AXIS**2 - B_SEMIMINOR_AXIS**2) / B_SEMIMINOR_AXIS**2

# eccentricity squared
ESQ = 2 * F_FLATTENING - F_FLATTENING**2

def calculateNlat(lat):
    return A_SEMIMAJOR_AXIS/np.sqrt(1-(ESQ * np.sin(np.radians(lat))**2))

def lla2ecef(lat, lon, alt):
    #converts ellipsoidal coordinates into cartesian ecef
    #lat in decimal degrees
    #lon in decimal degrees
    #alt in meters
    # check for bad values
    #if lat < -90 or lat > 90 or lon < -180 or lon > 180:
    #    return None

    N = calculateNlat(lat)
    x = (N + alt) * np.cos(np.radians( lat ) ) * np.cos( np.radians( lon ) )
    y = (N + alt) * np.cos(np.radians( lat ) ) * np.sin( np.radians( lon ) )
    z = ((N * (1 - ESQ)) + alt) * np.sin(np.radians((lat)))
    return x, y, z

def pcshow(pcl):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=512, height=512)
    opt = vis.get_render_option()
    opt.point_size = 2
    for p in pcl:
        vis.add_geometry(p)
    vis.run()
    vis.destroy_window()


def load_bin(velo_filename):
    scan = np.fromfile(velo_filename, dtype=np.float32)
    scan = scan.reshape((-1, 7))
    return scan

if __name__ == "__main__":

    pc='./tianjinport.txt'
    with open(pc,'r') as f:
        pcs=f.readlines()
    pcd=[]
    for line in pcs:
        [la,lo,al,_]=line.strip('\n').split(' ')
        la=float(la)
        lo=float(lo)
        al=float(al)
        x,y,z=lla2ecef(la,lo,al)
        pcd.append([x,y,z])
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(pcd)
    pcshow([pointcloud])
