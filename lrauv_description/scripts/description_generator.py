#!/usr/bin/env python3

# Copyright (C) 2021 Open Source Robotics Foundation
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

# Usage:
#   description_generator.py <input_sdf_file> <output_file>
# This file takes in the model.sdf.in file and generates an output model.sdf.
# The aim is to produce a perfectly stable model, despite of adding control
# elements.

import xml.etree.ElementTree as ET
import numpy as np
import sys
import os
import os.path as path

## These are parameters of the WHOLE VEHICLE
# TODO(arjo): Implement inertia. Need to implement Inertial<T>::operator-(const Inertial<T>&) first
total_mass = 147.5671 # Total mass of the vehicle
buoyancy_z_offset = 0.007 # Buoyancy offset
fluid_density = 1025 #fluid density

def read_pose(element):
    """ Read pose element."""
    return [float(token) for token in element.text.split()]

def write_float_array(pose):
    res = ""
    for p in pose:
        res += str(p) + " "
    return res


def calculate_center_of_mass(total_mass, template_path, output_path):

    sdf_file_input = ET.parse(template_path)

    for sdf in sdf_file_input.iter("sdf"):
        for model in sdf.iter("model"):
            moments = []
            main_body_com_tag = None
            main_body_com_pose = None
            collision_tag = None
            skipped_masses = []
            ## Get all links
            for links in model.iter("link"):
                link_pose = links.find(".pose")
                if link_pose is None:
                    center_of_mass = [0] * 6
                else:
                    # TODO(arjo) generate errors, perform protection
                    center_of_mass = [float(token) for token in link_pose.text.split()]

                ## Get inertial value of part
                inertia_element = links.find(".inertial")
                collision_element = links.find(".collision")

                ## Calculate buoyancy
                if collision_element is not None:
                    if collision_element.text.strip() == "@calculated":
                        collision_tag = collision_element
                    elif collision_element.text.strip() == "@neutral_buoyancy":
                        collision_element.text = ""
                        geometry = ET.SubElement(collision_element, "geometry")
                        box = ET.SubElement(geometry, "box")
                        size = ET.SubElement(box, "size")
                        mass = links.find(".inertial/mass")
                        if mass is None:
                            raise Exception("Mass not found")

                        sz = float(mass.text) / (0.1 * 0.1 * fluid_density)
                        size.text = write_float_array([0.1, 0.1, sz])
                        skipped_masses.append(mass)
                        continue
                    else:
                        # TODO(arjo): Calculate moments arising from collision
                        continue

                ## Calculate moments due to inertia
                if inertia_element is not None:
                    # TODO(arjo): Just put a @calculated on the whole inertia itself
                    mass = links.find(".inertial/mass")
                    pose = links.find(".inertial/pose")
                    if mass.text.strip() == "@calculated":
                        main_body_com_tag = mass
                        main_body_com_pose = pose
                        continue
                    else:
                        # add the moment into our moment list
                        if pose is not None:
                            center_of_mass = [float(token) for token in pose.text.split()]
                    assert len(center_of_mass) == 6
                    moments.append((float(mass.text), np.array([center_of_mass[0]] + [0]*5)))

            # get the moments in the model
            total_moment =  sum([mass * com for mass, com in moments])

            # this is the mass of the various other payloads
            component_mass = sum([mass for mass, _ in moments])
            remaining_mass = total_mass - component_mass
            main_body_com = - total_moment / remaining_mass

            main_body_com_tag.text = str(remaining_mass)
            main_body_com_pose.text = write_float_array([main_body_com[0], 0, 0, 0, 0, 0])

            # calculate buoyancy position
            cube_length = total_mass / (2 * 0.4 * fluid_density)
            collision_tag.text = ""
            pose_tag = ET.SubElement(collision_tag, "pose")
            pose_tag.text = write_float_array([0, 0, buoyancy_z_offset, 0, 0, 0])

            geometry = ET.SubElement(collision_tag, "geometry")
            box = ET.SubElement(geometry, "box")
            size = ET.SubElement(box, "size")
            size.text = write_float_array([2, 0.4, cube_length])

            assert (total_moment + main_body_com * remaining_mass == np.array([0]*6)).all()

    dir_name = path.dirname(output_path)
    if dir_name != '' and not path.exists(dir_name):
        os.makedirs(dir_name)

    sdf_file_input.write(output_path)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage:")
        print("description_generator.py <infile> <outfile>")
        exit(-100)

    calculate_center_of_mass(total_mass, sys.argv[1], sys.argv[2])
