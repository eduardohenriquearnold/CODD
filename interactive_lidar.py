import argparse
import logging
import random
import time
import threading
import numpy as np
from mayavi import mlab
import fixpath
import carla

def eventLoop(world):
    while True:
        time.sleep(0.005)
        world.tick()

def lidar_callback(data, buf):
    data = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype('f4')))
    points = np.reshape(data, (int(data.shape[0] / 4), 4))
    buf['pts'] = points[:,:3]
    buf['intensity'] = points[:,-1]

def main(args):
    logging.basicConfig(format='%(message)s', level=logging.INFO)

    try:
        #Load client & world
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.load_world(args.map)

        #Set configs
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        delta = 0.05
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        #  settings.no_rendering_mode = arg.no_rendering
        world.apply_settings(settings)

        #Set vehicle
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)

        spawn_points = world.get_map().get_spawn_points()
        if len(spawn_points) == 0:
            logging.error('No spawn points found')
        ego_transform = random.choice(spawn_points)
        ego_vehicle = world.spawn_actor(ego_bp,ego_transform)

        logging.info('Ego is spawned')
        ego_vehicle.set_autopilot(args.no_autopilot)

        #set lidar
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('dropoff_general_rate', '0.0')
        lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
        lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        lidar_bp.set_attribute('points_per_second', str(args.points_per_second))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
        lidar_bp.set_attribute('channels', str(args.channels))
        lidar_bp.set_attribute('range', str(args.range))
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8))
        lidar= world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)

        #spawn other vehicles
        spawnPoints = world.get_map().generate_waypoints(8) # waypoints every x meters 
        spawnPoints = [sp for sp in spawnPoints if sp.transform.location.distance(ego_transform.location) < args.range]
        spawnPoints = [sp for sp in spawnPoints if sp.transform.location.distance(ego_transform.location) > 3]
        spawnPoints = random.sample(spawnPoints, args.nvehicles)
        blueprints = world.get_blueprint_library().filter('vehicle')
        blueprints = random.choices(list(blueprints), k=args.nvehicles)
        vehicles = []
        for bp, sp in zip(blueprints, spawnPoints):
            transform = sp.transform
            transform.location.z = 0.65
            v = world.try_spawn_actor(bp, transform)
            if v is not None:
                v.set_autopilot(args.no_autopilot)
                vehicles.append(v)
        logging.info(f'Spawned {len(vehicles)} other vehicles')

        fig = mlab.figure(size=(960,540), bgcolor=(0.05,0.05,0.05))
        vis = mlab.points3d(0, 0, 0, 0, mode='point', figure=fig)
        mlab.view(distance=25)
        buf = {'pts': np.zeros((1,3)), 'intensity':np.zeros(1)}

        lidar.listen(lambda data : lidar_callback(data, buf))
        t1 = threading.Thread(target=eventLoop, args=[world], daemon=True)
        t1.start()

        @mlab.animate(delay=100)
        def anim():
            while True:
                vis.mlab_source.reset(x=buf['pts'][:,0], y=buf['pts'][:,1], z=buf['pts'][:,2], scalars=buf['intensity'])
                yield

        anim()
        mlab.show()

    finally:
        lidar.destroy()
        ego_vehicle.destroy()

        for v in vehicles:
            v.destroy()


if __name__ == '__main__':
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
        '--nvehicles',
        default=0,
        type=int,
        help='number of vehicles in the environment (default: 0)')
    argparser.add_argument(
        '--no-autopilot',
        action='store_false',
        help='disables the autopilot so the vehicle will remain stopped')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        pass
    finally:
        logging.info('Finished simulation')
