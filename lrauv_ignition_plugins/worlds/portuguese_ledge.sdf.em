<?xml version="1.0" ?>
<!--
  Development of this module has been funded by the Monterey Bay Aquarium
  Research Institute (MBARI) and the David and Lucile Packard Foundation
-->

<!--

  This world doesn't contain any vehicles. They're spawned at runtime by
  WorldCommPlugin as it receives LRAUVInit messages.

-->

<!--
   Tile heights calculated manually with

   $ gdalinfo -mm PortugueseLedgeTile1_DecDeg.grd | grep -i min
   Computed Min/Max=-98.637,-73.297
 -->

@{

import math
from dataclasses import dataclass
from ignition.math import SphericalCoordinates, Vector3d, Angle

fuel_model_url = "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Portuguese Ledge"

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
  <world name="LRAUV">
    <scene>
      <ambient>0.0 1.0 1.0</ambient>
      <background>0.0 0.7 0.8</background>

      <grid>false</grid>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>

      <!-- Center of Tile 1 -->
      <latitude_deg>@(tiles[0].lat_deg)</latitude_deg>
      <longitude_deg>@(tiles[0].lon_deg)</longitude_deg>

      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <physics name="1ms" type="dart">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <plugin
      filename="ignition-gazebo-buoyancy-system"
      name="ignition::gazebo::systems::Buoyancy">
      <uniform_fluid_density>1025</uniform_fluid_density>
    </plugin>

    <plugin
      filename="ignition-gazebo-particle-emitter2-system"
      name="ignition::gazebo::systems::ParticleEmitter2">
    </plugin>

    <plugin
      filename="WorldCommPlugin"
      name="tethys::WorldCommPlugin">
      <init_topic>/lrauv/init</init_topic>
    </plugin>

    <plugin name="ignition::gazebo" filename="dummy">

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

    <gui fullscreen="0">

      <!-- 3D scene -->
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>

        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <!-- looking at robot -->
        <camera_pose>0 6 6 0 0.5 -1.57</camera_pose>
        <!-- looking at all science data for 2003080103_mb_l3_las.csv -->
        <!--camera_pose>-50000 -30000 250000 0 1.1 1.58</camera_pose-->
        <camera_clip>
          <!-- ortho view needs low near clip -->
          <!-- but a very low near clip messes orbit's far clip ?! -->
          <near>0.1</near>
          <!-- See 3000 km away -->
          <far>3000000</far>
        </camera_clip>
      </plugin>

      <!-- Plugins that add functionality to the scene -->
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <ignition-gui>
          <property key="state" type="string">floating</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <ignition-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <ignition-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <ignition-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
        <warn_on_action_failure>false</warn_on_action_failure>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <ignition-gui>
          <anchors target="Select entities">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <ignition-gui>
          <anchors target="Select entities">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>

      <!-- World control -->
      <plugin filename="WorldControl" name="World control">
        <ignition-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>

          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>

        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>true</start_paused>
      </plugin>

      <!-- World statistics -->
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>

          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>

        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>

      <plugin filename="Plot3D" name="Plot 3D">
        <ignition-gui>
          <title>Plot Tethys 3D path</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
        <entity_name>tethys</entity_name>
        <color>0 0 1</color>
        <maximum_points>10000</maximum_points>
        <minimum_distance>0.5</minimum_distance>
      </plugin>
      <plugin filename="ComponentInspector" name="Component Inspector">
        <ignition-gui>
          <title>Inspector</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
      </plugin>
      <plugin filename="PointCloud" name="Visualize science data">
        <ignition-gui>
          <title>Visualize science data</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
      </plugin>
      <plugin filename="ViewAngle" name="Camera controls">
        <ignition-gui>
          <title>Camera controls</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
      </plugin>
      <plugin filename="GridConfig" name="Grid config">
        <ignition-gui>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
        <insert>
          <!-- 300 km x 300 km -->
          <cell_count>6</cell_count>
          <vertical_cell_count>0</vertical_cell_count>
          <!-- 50 km -->
          <cell_length>50000</cell_length>
          <pose>0 100000 0  0 0 0.32</pose>
          <color>0 1 0 1</color>
        </insert>
        <insert>
          <!-- 0.1 km x 0.1 km -->
          <cell_count>100</cell_count>
          <vertical_cell_count>0</vertical_cell_count>
          <!-- 1 m -->
          <cell_length>1</cell_length>
          <pose>0 0 0  0 0 0</pose>
          <color>0.5 0.5 0.5 1</color>
        </insert>
      </plugin>
      <plugin filename="ControlPanelPlugin" name="Tethys Controls">
        <ignition-gui>
          <title>Tethys controls</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
      </plugin>
      <!-- Sensor Data Map -->
      <plugin filename="WorldConfigPlugin" name="Environmental Configuration">
        <ignition-gui>
          <title>Environmental Configuration</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
      </plugin>
      <plugin filename="ReferenceAxis" name="Reference axis">
        <ignition-gui>
          <title>Reference axis</title>
          <property type="string" key="state">docked_collapsed</property>
        </ignition-gui>
        <fsk>tethys</fsk>
      </plugin>
    </gui>

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

  </world>
</sdf>
