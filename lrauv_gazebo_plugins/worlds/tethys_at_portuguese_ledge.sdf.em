<?xml version="1.0" ?>
<!--
  Development of this module has been funded by the Monterey Bay Aquarium
  Research Institute (MBARI) and the David and Lucile Packard Foundation
-->

<!--

  This world contains a single LRAUV vehicle, tethys.

-->

@{

import math
from dataclasses import dataclass
from gz.math7 import SphericalCoordinates, Vector3d, Angle

fuel_model_url = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Portuguese Ledge"

@dataclass
class Tile:
    index: int
    lat_deg: float
    lon_deg: float
    height: float
    pos_enu: Vector3d = Vector3d()

# Center of all 18 tiles in degrees
tiles = [
    Tile(1, 36.693509, -121.936568, 25.34),
    Tile(2, 36.693583, -121.944962, 27.094),
    Tile(3, 36.693658, -121.953356, 11.602),
    Tile(4, 36.693731, -121.961751, 6.781),
    Tile(5, 36.693804, -121.970145, 6.689),
    Tile(6, 36.693876, -121.978539, 30.707),
    Tile(7, 36.700269, -121.936475, 20.746),
    Tile(8, 36.700343, -121.944870, 29.343),
    Tile(9, 36.700418, -121.953265, 6.851),
    Tile(10, 36.700491, -121.961660, 6.462),
    Tile(11, 36.700564, -121.970055, 29.339),
    Tile(12, 36.700636, -121.978450, 148.439),
    Tile(13, 36.707029, -121.936382, 6.799),
    Tile(14, 36.707103, -121.944777, 6.814),
    Tile(15, 36.707178, -121.953173, 8.834),
    Tile(16, 36.707251, -121.961569, 11.934),
    Tile(17, 36.707324, -121.969965, 75.378),
    Tile(18, 36.707396, -121.978360, 229.765)]

# Convert to world ENU coordinates
sc = SphericalCoordinates(
    SphericalCoordinates.EARTH_WGS84,
    Angle(math.radians(tiles[0].lat_deg)),
    Angle(math.radians(tiles[0].lon_deg)),
    0, Angle(0))

for tile in tiles:
    vec = Vector3d(math.radians(tile.lat_deg), math.radians(tile.lon_deg), 0)
    pos_enu = sc.position_transform(vec,
        SphericalCoordinates.SPHERICAL,
        SphericalCoordinates.LOCAL2)
    tile.pos_enu = pos_enu

}@

<sdf version="1.6">
  <world name="portuguese_ledge">
    <scene>
      <ambient>0.0 1.0 1.0</ambient>
      <background>0.0 0.7 0.8</background>

      <grid>false</grid>
    </scene>
    
    <physics name="1ms" type="dart">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics> 

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>

      <!-- Center of Tile 1 -->
      <latitude_deg>@(tiles[0].lat_deg)</latitude_deg>
      <longitude_deg>@(tiles[0].lon_deg)</longitude_deg>

      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <plugin
      filename="gz-sim-environment-preload-system"
      name="gz::sim::systems::EnvironmentPreload">
      <data>../data/2003080103_mb_l3_las_1x1km.modded.csv</data>
      <dimensions>
        <time>elapsed_time_second</time>
        <space reference="spherical">
          <x>latitude_degree</x>
          <y>longitude_degree</y>
          <z>altitude_meter</z>
        </space>
      </dimensions>
    </plugin>

    <plugin
      filename="ScienceSensorsSystem"
      name="tethys::ScienceSensorsSystem">
      <data_path>2003080103_mb_l3_las.csv</data_path>
    </plugin>

    <include>
      <pose>0 0 0 0 0 0</pose>
      <uri>tethys_equipped</uri>
    </include>

    <plugin name="gz::sim" filename="dummy">
@[for tile in tiles]@
      <level name="level_@(tile.index)">
        <pose>@(tile.pos_enu.x()) @(tile.pos_enu.y()) @(tile.pos_enu.z()) 0 0 0</pose>
        <geometry>
          <box>
            <size>1000 1000 1000</size>
          </box>
        </geometry>
        <ref>portuguese_ledge_tile_@(tile.index)</ref>
      </level>
@[end for]@
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

@[for tile in tiles]@
    <model name="portuguese_ledge_tile_@(tile.index)">
      <static>true</static>
      <link name="link">
        <!-- Collisions seem to be misbehaving at the moment. Revisit if they're ever needed -->
        <!--collision name="collision">
          <geometry>
            <heightmap>
              <pos>@(tile.pos_enu)</pos>
              <uri>@(fuel_model_url)/tip/files/meshes/PortugueseLedgeTile@(tile.index)_DecDeg.nc</uri>
              <size>1000 1000 @(tile.height)</size>
            </heightmap>
          </geometry>
        </collision-->
        <visual name="visual">
          <geometry>
            <heightmap>
              <pos>@(tile.pos_enu.x()) @(tile.pos_enu.y()) @(tile.pos_enu.z()) 0 0 0</pos>
              <use_terrain_paging>true</use_terrain_paging>
              <texture>
                <diffuse>@(fuel_model_url)/tip/files/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>@(fuel_model_url)/tip/files/materials/textures/flat_normal.png</normal>
                <size>10</size>
              </texture>
              <uri>@(fuel_model_url)/tip/files/meshes/PortugueseLedgeTile@(tile.index)_DecDeg.nc</uri>
              <size>1000 1000 @(tile.height)</size>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
@[end for]@

    <!-- Uncomment for particle effect
      Requires ParticleEmitter2 in gz-sim 4.8.0, which will be copied
      to ParticleEmitter in Gazebo G.
      See https://github.com/gazebosim/gz-sim/pull/730 -->
    <!--include>
      <pose>-5 0 0 0 0 0</pose>
      <uri>turbidity_generator</uri>
    </include-->
  </world>
</sdf>
