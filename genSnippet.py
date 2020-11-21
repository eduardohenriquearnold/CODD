import argparse
import logging
import random
import time
import threading
import weakref
from queue import Queue

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
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8))
        self.lidar= world.spawn_actor(self.get_lidar_bp(args), lidar_transform, attach_to=self.vehicle)
        self.lidar.listen(lambda data : self.lidar_callback(data))

    def get_random_blueprint(self):
        blueprints = self.world.get_blueprint_library().filter('vehicle')
        return random.choice(blueprints)

    def get_lidar_bp(self, args):
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('dropoff_general_rate', '0.0')
        lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
        lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        lidar_bp.set_attribute('points_per_second', str(args.points_per_second))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / args.delta))
        lidar_bp.set_attribute('channels', str(args.channels))
        lidar_bp.set_attribute('range', str(args.range))
        return lidar_bp

    def lidar_callback(self, data):
        points = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype('f4')))
        point_cloud = np.reshape(points, (int(points.shape[0] / 4), 4))
        Vehicle.sensorQueue.put((data.frame, self.id, point_cloud, data.transform))

    def destroy(self):
        self.lidar.destroy()
        self.vehicle.destroy()

def main(args):
    try:
        #Load client & world
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.load_world(args.map)

        #Set configs
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        settings.fixed_delta_seconds = args.delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = args.no_rendering
        world.apply_settings(settings)

        #Spawn vehicles
        spawn_points = world.get_map().get_spawn_points()
        for _ in range(args.nvehicles):
            transform = random.choice(spawn_points)
            Vehicle(transform, world, args)

        #Main loop
        while(True):
            world.tick()
            snap = world.get_snapshot()
            
            try:
                for _ in range(len(Vehicle.instances)):
                    s = Vehicle.sensorQueue.get(True,5)
            except queue.Empty:
                logging.error(f'Missing sensor data for frame {snap.frame}!')

            logging.info(f'World frame {snap.frame}. Sensor data frame {s[0]} from all {len(Vehicle.instances)} vehicles')
            time.sleep(0.05)

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
        '--points-per-second',
        default=500000,
        type=int,
        help='lidar\'s points per second (default: 500000)')
    argparser.add_argument(
        '--delta',
        default=0.05,
        type=float,
        help='fixed simulation time-steps')
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

