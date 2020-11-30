import argparse
import logging
import random
import time
import threading
import weakref
from queue import Queue, Empty

import numpy as np
import h5py
from mayavi import mlab

import fixpath
import carla

class Vehicle:
    '''Creates and spawn a vehicle with a lidar sensor'''

    #Class variable that stores references to all instances
    instances = weakref.WeakSet()
    sensorQueue = Queue()

    def __init__(self, transform, world, args):
        '''Try to spawn vehicle at given transform, may fail due to collision. If it doesnt, spawns lidar sensor and add object to instances'''

        #try to spawn vehicle
        self.world = world
        self.vehicle = world.try_spawn_actor(self.get_random_blueprint(), transform)
        if self.vehicle is None:
            return
        Vehicle.instances.add(self)
        self.vehicle.set_autopilot(args.no_autopilot)
        self.id = self.vehicle.id

        #create lidar sensor and registers callback
        #lidar height is the height of the vehicle plus the necessary to avoid any points on the roof of the vehicle given the lower fov angle
        hp = max(self.vehicle.bounding_box.extent.x,self.vehicle.bounding_box.extent.y)*np.tan(np.radians(-args.lower_fov))
        lidar_transform = carla.Transform(carla.Location(z=2*self.vehicle.bounding_box.extent.z+hp))
        self.lidar= world.spawn_actor(self.get_lidar_bp(args), lidar_transform, attach_to=self.vehicle)
        self.lidar.listen(lambda data : self.lidar_callback(data))

    def get_random_blueprint(self):
        blueprints = self.world.get_blueprint_library().filter('vehicle')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        return random.choice(blueprints)

    def get_lidar_bp(self, args):
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('dropoff_general_rate', '0.0')
        lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
        lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        lidar_bp.set_attribute('points_per_second', str(args.points_per_cloud*args.fps))
        lidar_bp.set_attribute('rotation_frequency', str(args.fps))
        lidar_bp.set_attribute('channels', str(args.channels))
        lidar_bp.set_attribute('lower_fov', str(args.lower_fov))
        lidar_bp.set_attribute('range', str(args.range))
        return lidar_bp

    def lidar_callback(self, data):
        points = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype('f4')))
        point_cloud = np.reshape(points, (int(points.shape[0] / 4), 4))
        Vehicle.sensorQueue.put((data.frame, self.id, point_cloud, data.transform))

    def destroy(self):
        self.lidar.destroy()
        self.vehicle.destroy()

def transformPts(transform, pts, inverse=False):
    #split intensity from 3D coordinates, add homogeneus coordinate
    intensity = pts[:,-1,np.newaxis].copy()
    pts[:,-1] = 1

    #perform transformation
    mat = transform.get_inverse_matrix() if inverse else transform.get_matrix()
    mat = np.array(mat)
    ptst = pts @ mat.T

    #merge intensity back
    ptst = np.concatenate([ptst[:,:3],intensity], axis=1)
    return ptst

def carlaEventLoop(world, buf):
    while(True):
        world.tick()
        snap = world.get_snapshot()
        
        fusedPCL = []
        try:
            for _ in range(len(Vehicle.instances)):
                s = Vehicle.sensorQueue.get(True,5)
                pcl = s[2].astype(np.half)
                transform = s[3]
                pcl = transformPts(transform, pcl)
                fusedPCL.append(pcl)
        except Empty:
            logging.error(f'Missing sensor data for frame {snap.frame}!')

        fusedPCL = np.concatenate(fusedPCL, axis=0)
        buf['pts'] = fusedPCL

        logging.info(f'World frame {snap.frame}. Sensor data frame {s[0]} from all {len(Vehicle.instances)} vehicles')
        time.sleep(0.05)

def main(args):
    try:
        #Load client & world
        client = carla.Client(args.host, args.port)
        client.set_timeout(9.0)
        world = client.load_world(args.map)

        #Set configs
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        settings.fixed_delta_seconds = 1. / args.fps
        settings.synchronous_mode = True
        settings.no_rendering_mode = args.no_rendering
        world.apply_settings(settings)

        #Spawn vehicles (select one random point and only keep the points within the range - specificed according to lidar range)
        spawn_points = [waypoint.transform for waypoint in world.get_map().generate_waypoints(5)] # waypoints every x meters 
        sp_choice = random.choice(spawn_points)
        spawn_points = [sp for sp in spawn_points if sp.location.distance(sp_choice.location) < args.range/2]
        while(len(Vehicle.instances) < args.nvehicles):
            transform = random.choice(spawn_points)
            Vehicle(transform, world, args)

        fig = mlab.figure(size=(960,540), bgcolor=(0.05,0.05,0.05))
        vis = mlab.points3d(0, 0, 0, 0, mode='point', figure=fig)
        mlab.view(distance=25)
        buf = {'pts': np.zeros((1,4))}

        #Main loop
        t1 = threading.Thread(target=carlaEventLoop, args=[world,buf], daemon=True)
        t1.start()

        @mlab.animate(delay=100)
        def anim():
            while True:
                vis.mlab_source.reset(x=buf['pts'][:,0], y=buf['pts'][:,1], z=buf['pts'][:,2], scalars=buf['pts'][:,3])
                yield

        anim()
        mlab.show()
        
    finally:
        for v in Vehicle.instances:
            v.destroy()

if __name__ == '__main__':
    logging.basicConfig(format='%(message)s', level=logging.INFO)
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-m', '--map',
        metavar='M',
        default='Town03',
        type=str,
        help='Map name (default: Town03)')
    argparser.add_argument(
        '--channels',
        default=64.0,
        type=float,
        help='lidar\'s channel count (default: 64)')
    argparser.add_argument(
        '--range',
        default=100.0,
        type=float,
        help='lidar\'s maximum range in meters (default: 100.0)')
    argparser.add_argument(
        '--lower-fov',
        default=-25.0,
        type=float,
        help='lidar\'s lower vertical fov angle in degrees (default: -25.0)')
    argparser.add_argument(
        '--points-per-cloud',
        default=50000,
        type=int,
        help='lidar\'s points per measurement (default: 50000)')
    argparser.add_argument(
        '--fps',
        default=10.0,
        type=float,
        help='frames per second, define the fixed simulation time-steps. (default: 10fps)')
    argparser.add_argument(
        '--nvehicles',
        default=0,
        type=int,
        help='number of vehicles in the environment (default: 0)')
    argparser.add_argument(
        '--no-autopilot',
        action='store_false',
        help='disables the autopilot so the vehicle will remain stopped')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='use the no-rendering mode which will provide some extra'
        ' performance but you will lose the articulated objects in the'
        ' lidar, such as pedestrians')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        pass
    finally:
        logging.info('Finished simulation')

