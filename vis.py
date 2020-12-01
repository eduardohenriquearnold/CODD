import argparse
import numpy as np
import h5py
from mayavi import mlab

def getTransform(x, y, z, pitch, yaw, roll, degrees=True):
    '''Given location x,y,z and pitch, yaw, roll, obtain the matrix that convert from local to global CS using the left-handed system from UE4'''

    if degrees:
        pitch, yaw, roll = [np.radians(x) for x in [pitch, yaw, roll]]

    cy,sy = np.cos(yaw), np.sin(yaw)
    cr,sr = np.cos(roll), np.sin(roll)
    cp,sp = np.cos(pitch), np.sin(pitch)

    mat = np.array([cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr, x, \
                    cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr, y, \
                         sp,               -cp * sr,                 cp * cr, z, \
                          0,                      0,                       0, 0], dtype=np.float).reshape(4,4)

    return mat

def transformPoints(transformMatrix, pts, inverse=False):
    '''Given a transformation matrix [4,4] convert pts [N,3] or [N,4] (last coordinate is intensity)'''

    #Check if intensity is available
    if pts.shape[1] == 4:
        #split intensity from 3D coordinates, add homogeneus coordinate
        intensity = pts[:,-1,np.newaxis].copy()
        pts[:,-1] = 1
    else:
        #add homogeneus coordinate
        intensity = None
        pts = np.concatenate([pts, np.ones((pts.shape[0],1))], axis=1)

    #perform transformation
    mat = np.array(transformMatrix)
    if inverse:
        mat = np.linalg.inv(mat)
    ptst = pts @ mat.T
    ptst = ptst[:,:3]

    #merge intensity back
    if intensity is not None:
        ptst = np.concatenate([ptst,intensity], axis=1)

    return ptst

def updateBoundingBox(x,y,z,yaw,pitch,w,l,h, vis_bb):
    #Create 8 corner points
    cpts = 0.5*np.array([[-1,1,-1],[1,1,-1],[1,-1,-1],[-1,-1,-1],[-1,1,1],[1,1,1],[1,-1,1],[-1,-1,1]]) + np.array([[0,0,0.5]])
    cpts *= np.array([[w,l,h]])
    cpts = transformPoints(getTransform(x,y,z,pitch,yaw,0), cpts)

    #list of 16 points to create whole BB
    pts = cpts[[0,3,7,3,2,6,2,1,5,1,0,4,7,6,5,4],:]

    #update vis
    vis_bb.mlab_source.reset(x=pts[:,0], y=pts[:,1], z=pts[:,2])

def main(args):
    #Load file/data
    f = h5py.File(args.filename, 'r')
    pcls = f['point_cloud']
    lidar_pose = f['lidar_pose']
    bbs = f['vehicle_boundingbox']
    
    nframes = pcls.shape[0]
    nvehicles = pcls.shape[1]

    #Create Mayavi Visualisation
    fig = mlab.figure(size=(960,540), bgcolor=(0.05,0.05,0.05))
    zeros = np.zeros(pcls.shape[1]*pcls.shape[2])
    vis = mlab.points3d(zeros, zeros, zeros, zeros, mode='point', figure=fig)
    zeros = np.zeros(16)
    vis_bbs = [mlab.plot3d(zeros, zeros, zeros, zeros, color=(0,1,0), tube_radius=None, line_width=1, figure=fig) for i in range(nvehicles)]

    #Iterate through frames
    @mlab.animate(delay=100)
    def anim():
        for frame in range(nframes):
            print(f'Frame {frame}')

            fusedPCL = []
            for i in range(nvehicles):
                #Get PCL for the given vehicle in the global Coordinate System
                pcl = pcls[frame,i]
                transform = getTransform(*lidar_pose[frame,i].tolist())
                pcl_global = transformPoints(transform, pcl)
                fusedPCL.append(pcl_global)

                #Update the vehicle BB visualisation
                updateBoundingBox(*bbs[frame,i].tolist(), vis_bbs[i])

            #Update PCL visualisation with Mayavi
            fusedPCL = np.concatenate(fusedPCL, axis=0)
            vis.mlab_source.set(x=fusedPCL[:,0], y=fusedPCL[:,1], z=fusedPCL[:,2], scalars=fusedPCL[:,3])
            yield

    anim()
    mlab.show()

if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        'filename',
        type=str,
        help='Path to snippet to be visualised')

    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        pass
    finally:
        print('Finished visualisation!')
