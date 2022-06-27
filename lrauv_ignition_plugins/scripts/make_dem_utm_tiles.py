#!/usr/bin/env python3

# Copyright (C) 2022 Monterey Bay Aquarium Research Institute (MBARI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation

import subprocess

import netCDF4 as nc
import numpy as np
from pyproj import Proj


def mk_tiles(map_path: str, utm_zone: int, out_path: str, tile_size: int, overlap: int):
    """
    Generates digital elevation map (DEM) tiles from parent DEM using Generic Mapping Tool (GMT).

    To install GMT,see: https://www.generic-mapping-tools.org

    Tiles are generated with size of tile_size x tile_size, and with "overlap"
    meters shared with adjacent tiles on each edge.

    Tile boundaries will be computed starting at the bottom/right corner of
    the parent DEM (rounded to nearest 100 m).

    :param map_path: file path to full-resolution DEM
    :param utm_zone: full-res DEM UTM zone
    :param out_path: output path for tile files
    :param tile_size: meters, each tile will have are of tile_size x tile_size
    :param overlap: meters, tiles will have "overlap" meters shared with adjacent tiles on each edge
    :return: void
    """
    unique_size = tile_size - 2 * overlap

    # Open the file and create an instance of the ncCDF4 class
    nc_fid = nc.Dataset(map_path, 'r')

    # Extract data from NetCDF file
    x = nc_fid.variables['x'][:]
    y = nc_fid.variables['y'][:]
    z = nc_fid.variables['z'][:]

    # Find DEM boundary limits (to nearest 100)
    x_lim = [np.ceil(min(x) / 100) * 100, np.floor(max(x) / 100) * 100]
    y_lim = [np.ceil(min(y) / 100) * 100, np.floor(max(y) / 100) * 100]

    # Arrange DEM tile corners
    x_right_corner = np.arange(x_lim[1], x_lim[0] + tile_size, -unique_size - overlap).tolist()
    y_bottom_corner = np.arange(y_lim[0], y_lim[1] - tile_size, unique_size + overlap).tolist()
    # Combine to create the tile grid
    [A, B] = np.meshgrid(x_right_corner, y_bottom_corner)
    C = np.concatenate((A.T, B.T), axis=1)
    bottom_right_corner = np.reshape(C, (int(C.size/2), 2), order='F')

    # Tile boundaries:
    left_bound  = bottom_right_corner[:, 0] - tile_size
    right_bound = bottom_right_corner[:, 0]
    lower_bound = bottom_right_corner[:, 1]
    upper_bound = bottom_right_corner[:, 1] + tile_size

    # Check the data
    assert all(left_bound >= x_lim[0]), "Tiles violate min X bound."
    assert all(right_bound <= x_lim[1]), "Tiles violate max X bound."
    assert all(lower_bound >= y_lim[0]), "Tiles violate min Y bound."
    assert all(upper_bound <= y_lim[1]), "Tiles violate max Y bound."

    # Tile centers:
    nTiles = len(bottom_right_corner)
    x_tile_centers = (left_bound + right_bound) / 2
    y_tile_centers = (lower_bound + upper_bound) / 2

    P = Proj(proj='utm', zone=utm_zone, ellps='WGS84', preserve_units=True)
    lon_tile_centers, lat_tile_centers = P(x_tile_centers, y_tile_centers, inverse=True)

    # Log tile centers
    tilesCenterFID = open(out_path + 'tile_centers.csv', 'w')
    tilesCenterFID.write('Tile, UTM_zone, Eastings, Northings, Lat, Lon\n')

    # Build gmt argument strings and loop through
    gmt_grdcut = '/usr/local/bin/gmt grdcut ' + map_path + ' '

    for i in range(nTiles):

        # Assemble grdcut argument string
        tile_name = 'PortugueseLedgeTile{}_UTM'.format(i)
        tile_path = '{}{}.grd '.format(out_path, tile_name)
        gcut_out_path = '-G{} '.format(tile_path)
        gcut_bounds = '-R{}/{}/{}/{}'.format(left_bound[i], right_bound[i], lower_bound[i], upper_bound[i])

        gmt_cmd = gmt_grdcut + gcut_out_path + gcut_bounds

        # Write to centers file
        tilesCenterFID.write(f'{tile_name}, {utm_zone}, '
                             f'{x_tile_centers[i]}, {y_tile_centers[i]}, '
                             f'{lat_tile_centers[i]}, {lon_tile_centers[i]}\n')

        print(gmt_cmd)
        subprocess.run([gmt_cmd], shell=True)

    tilesCenterFID.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(prog="make_tiles",
                                     description="Make digital elevation map (DEM) tiles from parent DEM.",
                                     epilog="run example: \n"
                                            "python3 make_tiles.py "
                                            "--dem_path 'PortugueseLedge_MAUV_Topo1m_UTM.grd' "
                                            "--utm_zone 10 --output_path 'data/'")

    parser.add_argument("-p", "--dem_path", default='', type=str,
                        help="path to parent DEM file", required=True)
    parser.add_argument("-z", "--utm_zone", default=0, type=int, required=True,
                        help="DEM utm zone")
    parser.add_argument("-o", "--output_path", default='', type=str,
                        help="tile files output path", required=True)
    parser.add_argument("-s", "--tile_size", default=1000.0, type=float,
                        help="tile size in meters,"
                             "each tile will have size of tile_size x tile_size")
    parser.add_argument("-l", "--tile_overlap", default=250.0, type=float,
                        help="tile overlap in meters, "
                             "tiles will have 'overlap' meters shared with adjacent tiles on each edge")

    args = parser.parse_args()

    mk_tiles(args.dem_path, args.utm_zone, args.output_path, args.tile_size, args.tile_overlap)
