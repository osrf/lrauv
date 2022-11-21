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

<sdf version="1.9">
  <world name="portuguese_ledge">
    <scene>
      <!-- For turquoise ambient to match particle effect -->
      <ambient>0.0 1.0 1.0</ambient>
      <!-- For default gray ambient -->
      <!--background>0.8 0.8 0.8</background-->
      <background>0.0 0.7 0.8</background>
    </scene>

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
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

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    
    <plugin
      filename="gz-sim-buoyancy-system"
      name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1025</default_density>
        <density_change>
          <above_depth>0.5</above_depth>
          <density>1.125</density>
        </density_change>
      </graded_buoyancy>
    </plugin>
    
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
       <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
    </plugin>
    
    <plugin
      filename="gz-sim-acoustic-comms-system"
      name="gz::sim::systems::AcousticComms">
      <max_range>2500</max_range>
      <speed_of_sound>1500</speed_of_sound>
    </plugin>
    
    <plugin
      filename="DopplerVelocityLogSystem"
      name="tethys::DopplerVelocityLogSystem">
    </plugin>
    
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
    
    <plugin
      filename="gz-sim-magnetometer-system"
      name="gz::sim::systems::Magnetometer">
    </plugin>
    
    <!--
      Requires ParticleEmitter2 in gz-sim 4.8.0, which will be copied
      to ParticleEmitter in Gazebo G.
      See https://github.com/gazebosim/gz-sim/pull/730
    -->
    <plugin
      filename="gz-sim-particle-emitter2-system"
      name="gz::sim::systems::ParticleEmitter2">
    </plugin>

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

    <gui fullscreen="0">
      <!-- 3D scene -->
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
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
        <gz-gui>
          <property key="state" type="string">floating</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <gz-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
        <warn_on_action_failure>false</warn_on_action_failure>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <gz-gui>
          <anchors target="Select entities">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <gz-gui>
          <anchors target="Select entities">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <!-- World control -->
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
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
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>true</start_paused>
      </plugin>
      <!-- World statistics -->
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
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
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
      <plugin filename="Plot3D" name="Plot 3D">
        <gz-gui>
          <title>Plot Tethys 3D path</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
        <entity_name>tethys</entity_name>
        <color>0 0 1</color>
        <maximum_points>10000</maximum_points>
        <minimum_distance>0.5</minimum_distance>
      </plugin>
      <plugin filename="ComponentInspector" name="Component Inspector">
        <gz-gui>
          <title>Inspector</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
      </plugin>
      <plugin filename="PointCloud" name="Visualize science data">
        <gz-gui>
          <title>Visualize science data</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
      </plugin>
      <plugin filename="ViewAngle" name="Camera controls">
        <gz-gui>
          <title>Camera controls</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
      </plugin>
      <plugin filename="GridConfig" name="Grid config">
        <gz-gui>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
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
        <gz-gui>
          <title>Tethys controls</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
      </plugin>
      <plugin filename="SpawnPanelPlugin" name="Spawn LRAUV Panel">
        <gz-gui>
          <title>Spawn LRAUVs</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
      </plugin>
      <plugin filename="ReferenceAxis" name="Reference axis">
        <gz-gui>
          <title>Reference axis</title>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
        <fsk>tethys</fsk>
      </plugin>
    </gui>

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

    <!-- Uncomment if you need a ground plane -->
    <!-- <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model> -->

    <!-- Particle effect
      Requires ParticleEmitter2 in gz-sim 4.8.0, which will be copied
      to ParticleEmitter in Gazebo G.
      See https://github.com/gazebosim/gz-sim/pull/730 -->
    <!--include>
      <pose>-5 0 0 0 0 0</pose>
      <uri>turbidity_generator</uri>
    </include-->

    <include>
      <pose>0 0 0 0 0 3.14</pose>
      <uri>tethys_equipped</uri>
    </include>

    <include>
      <pose>5 0 0 0 0 3.14</pose>
      <uri>tethys_equipped</uri>
      <name>triton</name>

      <experimental:params>
        <sensor element_id="base_link::salinity_sensor" action="modify">
          <topic>/model/triton/salinity</topic>
        </sensor>
        <sensor element_id="base_link::temperature_sensor" action="modify">
          <topic>/model/triton/temperature</topic>
        </sensor>
        <sensor element_id="base_link::chlorophyll_sensor" action="modify">
          <topic>/model/triton/chlorophyll</topic>
        </sensor>
        <sensor element_id="base_link::current_sensor" action="modify">
          <topic>/model/triton/current</topic>
        </sensor>
        <plugin element_id="gz::sim::systems::Thruster" action="modify">
          <namespace>triton</namespace>
        </plugin>
        <plugin element_id="tethys::TethysCommPlugin" action="modify">
          <namespace>triton</namespace>
          <command_topic>triton/command_topic</command_topic>
          <state_topic>triton/state_topic</state_topic>
        </plugin>
        <plugin element_id="gz::sim::systems::BuoyancyEngine" action="modify">
          <namespace>triton</namespace>
        </plugin>
        <plugin element_id="gz::sim::systems::DetachableJoint" action="modify">
          <topic>/model/triton/drop_weight</topic>
        </plugin>
        <plugin element_id="gz::sim::systems::CommsEndpoint" action="modify">
          <address>2</address>
          <topic>2/rx</topic>
        </plugin>
        <plugin element_id="tethys::RangeBearingPlugin" action="modify">
          <address>2</address>
          <namespace>triton</namespace>
        </plugin>
      </experimental:params>

    </include>

    <include>
      <pose>-5 0 0 0 0 3.14</pose>
      <uri>tethys_equipped</uri>
      <name>daphne</name>

      <experimental:params>
        <sensor element_id="base_link::salinity_sensor" action="modify">
          <topic>/model/daphne/salinity</topic>
        </sensor>
        <sensor element_id="base_link::temperature_sensor" action="modify">
          <topic>/model/daphne/temperature</topic>
        </sensor>
        <sensor element_id="base_link::chlorophyll_sensor" action="modify">
          <topic>/model/daphne/chlorophyll</topic>
        </sensor>
        <sensor element_id="base_link::current_sensor" action="modify">
          <topic>/model/daphne/current</topic>
        </sensor>
        <plugin element_id="gz::sim::systems::Thruster" action="modify">
          <namespace>daphne</namespace>
        </plugin>
        <plugin element_id="tethys::TethysCommPlugin" action="modify">
          <namespace>daphne</namespace>
          <command_topic>daphne/command_topic</command_topic>
          <state_topic>daphne/state_topic</state_topic>
        </plugin>
        <plugin element_id="gz::sim::systems::BuoyancyEngine" action="modify">
          <namespace>daphne</namespace>
        </plugin>
        <plugin element_id="gz::sim::systems::DetachableJoint" action="modify">
          <topic>/model/daphne/drop_weight</topic>
        </plugin>
        <plugin element_id="gz::sim::systems::CommsEndpoint" action="modify">
          <address>3</address>
          <topic>3/rx</topic>
        </plugin>
        <plugin element_id="tethys::RangeBearingPlugin" action="modify">
          <address>3</address>
          <namespace>daphne</namespace>
        </plugin>
      </experimental:params>

    </include>

    <!-- Swimming race lane signs -->
    <include>
      <pose>0 0 -1 0 0 3.1415926</pose>
      <uri>ABCSign_5m</uri>
      <name>start_line</name>
    </include>
    <include>
      <pose>0 -25 -1 0 0 3.1415926</pose>
      <uri>ABCSign_5m</uri>
      <name>finish_line</name>
    </include>

  </world>
</sdf>
