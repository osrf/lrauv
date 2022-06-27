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

from os import listdir
from os.path import join

import netCDF4 as nc
from pyproj import Proj


def utm2deg(northings, eastings, utm_zone):
    """
    Converts Universal Transverse Mercator (UTM) northings
    and eastings coordinates to WGS84 Lat/Lon decimal degree.

    :param northings: UTM northings
    :param eastings: UTM eastings
    :param utm_zone: UTM grid zone number
    :return WGS84 Lat/Lon in decimal degree
    """
    P = Proj(proj='utm', zone=utm_zone, ellps='WGS84', preserve_units=True)
    lon, lat = P(northings, eastings, inverse=True)
    return lon, lat


def utm_tile_to_latlon(tile_path: str, utm_zone: int):
    """
    Converts Universal Transverse Mercator (UTM) tiles to
    WGS84 Lat/Lon decimal degree and exports to new grd file (NetCDF4).

    :param tile_path: path to UTM tile file
    :param utm_zone: tile UTM zone
    """

    # Open the file and create an instance of the ncCDF4 class
    nc_utm = nc.Dataset(tile_path, 'r')

    # Extract data from NetCDF file
    x = nc_utm.variables['x'][:]
    y = nc_utm.variables['y'][:]

    lon, lat = utm2deg(x, y, utm_zone)

    nc_deg = nc.Dataset(tile_path.replace(".grd", "_DecDeg.grd"), 'w')

    # Write attributes
    nc_deg.title = nc_utm.title
    nc_deg.description = nc_utm.description
    nc_deg.Conventions = nc_utm.Conventions
    nc_deg.GMT_version = nc_utm.GMT_version
    nc_deg.history = nc_utm.history

    # Write dimensions
    nc_deg.createDimension('lat', nc_utm.dimensions['x'].size)
    nc_deg.createDimension('lon', nc_utm.dimensions['y'].size)
    nc_deg.createDimension('grid_mapping', nc_utm.dimensions['grid_mapping'].size)

    # Write variables
    longitude = nc_deg.createVariable("lon", "f8", ("lon",),
                                      zlib=True, complevel=3,  # config compression to match the UTM files
                                      fill_value=False)
    longitude.long_name = "Longitude (degrees)"
    longitude.units = "degrees_east"
    longitude.standard_name = "longitude"
    longitude.axis = "X"
    longitude.actual_range = [min(lon), max(lon)]
    longitude[:] = lon

    latitude = nc_deg.createVariable("lat", "f8", ("lat",),
                                     zlib=True, complevel=3,
                                     fill_value=False)
    latitude.long_name = "Latitude (degrees)"
    latitude.units = "degrees_north"
    latitude.standard_name = "latitude"
    latitude.axis = "Y"
    latitude.actual_range = [min(lat), max(lat)]
    latitude[:] = lat

    z = nc_deg.createVariable("z", "f4", ("lon", "lat"),
                              zlib=True, complevel=3,
                              fill_value=False)
    z.long_name = nc_utm.variables['z'].long_name
    z.grid_mapping = nc_utm.variables['z'].grid_mapping
    z.actual_range = nc_utm.variables['z'].actual_range
    z[:] = nc_utm.variables['z'][:]

    grid_mapping = nc_deg.createVariable("grid_mapping", "S1", "grid_mapping")
    grid_mapping.spatial_ref = nc_utm['grid_mapping'].spatial_ref

    nc_deg.close()
    nc_utm.close()


if __name__ == '__main__':
    import argparse

    # parse command-line arguments
    parser = argparse.ArgumentParser(description='Converts tiles with Universal Transverse Mercator (UTM)'
                                                 ' coordinates to WGS84 Lat/Lon decimal degree.')

    parser.add_argument("-p", "--path", type=str, required=True,
                        help="path to tiles folder")
    parser.add_argument("-z", "--utm_zone", type=int, required=True,
                        help="DEM utm zone")

    args = parser.parse_args()

    grd_files = [join(args.path, f) for f in listdir(args.path) if f.endswith("UTM.grd")]

    for f in sorted(grd_files):
        print("Converting: {}".format(f))
        utm_tile_to_latlon(tile_path=f, utm_zone=args.utm_zone)
