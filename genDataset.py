import argparse
import logging
import numpy as np

import random
import sys
import subprocess
from genSnippet import * 

logging.basicConfig(format='%(message)s', level=logging.INFO)
argparser = argparse.ArgumentParser()
argparser.add_argument(
    'N',
    type=int,
    help='Number of snippets to generate')
opts = argparser.parse_args()

#Set parameters of simulation/distributions
binomial_vehicles = (20, 0.444) # corresponding to mean=9, var=5; we add 1 to guarantee nvehicles>0
binomial_pedestrians = (8, 0.5) # corresponding to mean=4, var=2; we add 1 to guarantee at least 1 pedestrian
uniform_seed = (0,1000)
maps = list(range(1,8)) + [10]
fps = 5
lidar_range = 100
lidar_channels = 64
points_per_cloud = 50000
nframes = 125
burn = 30

#Start generating snippets
for i in range(1,opts.N+1):
    args = argparse.Namespace()
    args.host = '127.0.0.1'
    args.port = 2000
    mapNumber = random.choice(maps)
    args.map = f'Town0{mapNumber}' if mapNumber<10 else 'Town10HD'
    args.channels = lidar_channels
    args.range = lidar_range
    args.lower_fov = -25
    args.points_per_cloud = points_per_cloud
    args.fps = fps
    args.nvehicles = np.random.binomial(*binomial_vehicles) + 1
    args.npedestrians = np.random.binomial(*binomial_pedestrians) + 1
    args.no_autopilot = True
    args.no_rendering = False
    args.seed = np.random.randint(*uniform_seed)
    args.save = f'data/m{mapNumber}v{args.nvehicles}p{args.npedestrians}s{args.seed}.hdf5'
    args.frames = nframes
    args.burn = burn
    print(args)

    logging.info(f'Started generating Snippet {i}/{opts.N}')
    subprocess.run(['python', 'genSnippet.py', '--map', args.map, '--channels',str(args.channels), '--range',str(args.range), '--lower-fov',str(args.lower_fov),'--points-per-cloud',str(args.points_per_cloud),'--fps',str(args.fps),'--nvehicles',str(args.nvehicles),'--npedestrians',str(args.npedestrians), '--frames', str(args.frames), '--burn', str(args.burn), '--save', args.save], stdout=sys.stdout)
    #  main(args)
    logging.info(f'Finished generating Snippet {i}/{opts.N}. Saved as {args.save}')
