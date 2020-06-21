# !/usr/bin/env python

import rospy
import numpy as np

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="stairs_collision0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000000.0</kp>
              <kd>1.0</kd>
              <min_depth>0.0</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="stairs_visual0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""
sdf_sand = """<?xml version='1.0'?>
<sdf version='1.6'>
    <model name="MODELNAME">
        <link name='link'>
            <pose frame=''>0 0 0.01 0 0 0 </pose>
            <inertial>
                <mass>1</mass>
                <inertia>
                    <ixx>0.1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.1</iyy>
                    <iyz>0</iyz>
                    <izz>0.1</izz>
                </inertia>
            </inertial>
            <visual name='visual'>
                <pose frame=''>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                        <scale>SIZEXYZ</scale>
                        <uri>model://sand/sand_particle.stl</uri>
                    </mesh>
                </geometry>
                <material>
                    <lighting>1</lighting>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Yellow</name>
                    </script>
                    <ambient>0.3 0.25 0.1 1</ambient>
                    <diffuse>0.7 0.6 0.4 1</diffuse>
                    <specular>0.01 0.005 0.001 1</specular>
                    <emissive>0 0 0 1</emissive>
                </material>
                <transparency>0</transparency>
                <cast_shadows>1</cast_shadows>
            </visual>
            <collision name='collision'>
                <laser_retro>0</laser_retro>
                <max_contacts>10</max_contacts>
                <pose frame=''>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                        <scale>SIZEXYZ</scale>
                        <uri>model://sand/sand_particle.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1</mu>
                            <mu2>1</mu2>
                            <fdir1>0 0 0</fdir1>
                            <slip1>0</slip1>
                            <slip2>0</slip2>
                        </ode>
                        <torsional>
                            <coefficient>1</coefficient>
                            <patch_radius>0</patch_radius>
                            <surface_radius>0</surface_radius>
                            <use_patch_radius>1</use_patch_radius>
                            <ode>
                                <slip>0</slip>
                            </ode>
                        </torsional>
                    </friction>
                    <bounce>
                        <restitution_coefficient>0.2</restitution_coefficient>
                        <threshold>1.01</threshold>
                    </bounce>
                    <contact>
                        <collide_without_contact>0</collide_without_contact>
                        <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
                        <collide_bitmask>1</collide_bitmask>
                        <ode>
                            <soft_cfm>0</soft_cfm>
                            <soft_erp>0.2</soft_erp>
                            <kp>1e+13</kp>
                            <kd>1</kd>
                            <max_vel>0.01</max_vel>
                            <min_depth>0</min_depth>
                        </ode>
                        <bullet>
                            <split_impulse>1</split_impulse>
                            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                            <soft_cfm>0</soft_cfm>
                            <soft_erp>0.2</soft_erp>
                            <kp>1e+13</kp>
                            <kd>1</kd>
                        </bullet>
                    </contact>
                </surface>
            </collision>
        </link>
        <static>0</static>
        <allow_auto_disable>1</allow_auto_disable>
    </model>
</sdf>
"""
sdf_sand_box = """<sdf version='1.6'>
  <model name='sand_box_osher'>
    <link name='sand_box_osher'>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <inertial>
        <pose frame=''>-0.35285 -0.305 0.11027 0 -0 0</pose>
        <mass>2000.892</mass>
        <inertia>
          <ixx>130.2204</ixx>
          <ixy>-220.5538e-15</ixy>
          <ixz>-4.85191</ixz>
          <iyy>276.363</iyy>
          <iyz>-77.9029e-15</iyz>
          <izz>135.62</izz>
        </inertia>
      </inertial>
      <collision name='sand_box_osher_collision'>
        <pose frame=''>0 0 0 1.5708 -0 0</pose>
        <geometry>
          <mesh>
            <scale>1 0.8 1</scale>
            <uri>model://sand_box_osher/meshes/sand_box_osher.STL</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name='sand_box_osher_visual'>
        <pose frame=''>0 0 0 1.5708 -0 0</pose>
        <geometry>
          <mesh>
            <scale>1 0.8 1</scale>
            <uri>model://sand_box_osher/meshes/sand_box_osher.STL</uri>
          </mesh>
        </geometry>
          <material>
                    <ambient>0.3 0.25 0.1 1</ambient>
                    <diffuse>0.7 0.6 0.4 1</diffuse>
                    <specular>0.01 0.005 0.001 1</specular>
                    <emissive>0 0 0 1</emissive>
  </material>
  	<transparency>0.5</transparency>

      </visual>
    </link>
  </model>
</sdf>
"""
sdf_unit_sphere = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name="MODELNAME">
    <link name='link'>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0000490147</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000049147</iyy>
          <iyz>0</iyz>
          <izz>0.000049147</izz>
        </inertia>
        <pose frame=''>0 0 0 0 -0 0</pose>
      </inertial>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>RADIUS</radius>
          </sphere>
        </geometry>
        <material>
                    <lighting>1</lighting>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Yellow</name>
                    </script>
                    <ambient>0.3 0.25 0.1 1</ambient>
                    <diffuse>0.7 0.6 0.4 1</diffuse>
                    <specular>0.01 0.005 0.001 1</specular>
                    <emissive>0 0 0 1</emissive>
        </material>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>RADIUS</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>
    <static>0</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
"""
sdf_sand2 = """<?xml version='1.0'?>
<sdf version='1.6'>
    <model name="MODELNAME">
        <link name='link'>
            <pose frame=''>0 0 0.01 0 0 0 </pose>
            <inertial>
                <mass>1</mass>
                <inertia>
                    <ixx>0.1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.1</iyy>
                    <iyz>0</iyz>
                    <izz>0.1</izz>
                </inertia>
            </inertial>
            <visual name='visual'>
                <pose frame=''>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                        <scale>SIZEXYZ</scale>
                        <uri>model://sand/sand_particle.stl</uri>
                    </mesh>
                </geometry>
                <material>
                    <lighting>1</lighting>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Yellow</name>
                    </script>
                    <ambient>0.3 0.25 0.1 1</ambient>
                    <diffuse>0.7 0.6 0.4 1</diffuse>
                    <specular>0.01 0.005 0.001 1</specular>
                    <emissive>0 0 0 1</emissive>
                </material>
                <transparency>0</transparency>
                <cast_shadows>1</cast_shadows>
            </visual>
            <collision name='collision'>
                <laser_retro>0</laser_retro>
                <max_contacts>10</max_contacts>
                <pose frame=''>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                        <scale>SIZEXYZ</scale>
                        <uri>model://sand/sand_particle.stl</uri>
                    </mesh>
                </geometry>
                <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>
    <static>0</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
"""

class Spawner:
    def __init__(self):

        self.px = 0
        self.py = 0
        self.pz = 0
        self.rr = 0
        self.rp = 0
        self.rz = 0
        self.sx = 0
        self.sy = 0
        self.sz = 0

    def create_cube_request(self,modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
        """Create a SpawnModelRequest with the parameters of the cube given.
        modelname: name of the model for gazebo
        px py pz: position of the cube (and it's collision cube)
        rr rp ry: rotation (roll, pitch, yaw) of the model
        sx sy sz: size of the cube"""
        cube = deepcopy(sdf_sand2)
        # Replace size of model
        size_str = str(round(sx, 3)) + " " + \
                   str(round(sy, 3)) + " " + str(round(sz, 3))
        cube = cube.replace('SIZEXYZ', size_str)
        # Replace modelname
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnModelRequest()
        req.model_name = modelname
        req.model_xml = cube
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz

        q = quaternion_from_euler(rr, rp, ry)
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req

    def create_sphere_request(self,modelname, px, py, pz, rr, rp, ry, r):
        """Create a SpawnModelRequest with the parameters of the cube given.
        modelname: name of the model for gazebo
        px py pz: position of the cube (and it's collision cube)
        rr rp ry: rotation (roll, pitch, yaw) of the model
        sx sy sz: size of the cube"""
        cube = deepcopy(sdf_unit_sphere)
        # Replace size of model
        cube = cube.replace('RADIUS', str(r))
        # Replace modelname
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnModelRequest()
        req.model_name = modelname
        req.model_xml = cube
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz

        q = quaternion_from_euler(rr, rp, ry)
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req

    def create_box_request(self,modelname, px, py, pz, rr, rp, ry):
        """Create a SpawnModelRequest with the parameters of the cube given.
        modelname: name of the model for gazebo
        px py pz: position of the cube (and it's collision cube)
        rr rp ry: rotation (roll, pitch, yaw) of the model"""
        cube = deepcopy(sdf_sand_box)

        req = SpawnModelRequest()
        req.model_name = modelname
        req.model_xml = cube
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz

        q = quaternion_from_euler(rr, rp, ry)
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req




