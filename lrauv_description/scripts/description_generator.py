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

def generate_model(template_path, output_path):
    """
    Parses the template_path file and generates a hydrostatically stable file on
    output_path.

    * Sets the collision volume for control surfaces and buoyancy engine so
      they're neutrally buoyant.
    * Sets the base link's volume so that it cancels out the mass from itself,
      the battery and the drop weight.
    * Sets the base link's mass and CoM position so that it balances the moments
      on the X axis from the battery and drop weight.

    :param template_path: Path to templated file
    :param output_path: Path to save the resulting file
    """

    sdf_file_input = ET.parse(template_path)

    # Assume only one model in SDF file
    sdf_tag = sdf_file_input.find(".")
    assert sdf_tag is not None

    model_tag = sdf_tag.find("model")
    assert model_tag is not None

    x_moments = []
    base_link_mass_tag = None
    base_link_inertial_pose_tag = None
    base_link_collision_tag = None

    ## Iterate over all links
    for link_tag in model_tag.iter("link"):

        ## Calculate buoyancy (collision volume) for links with collisions
        collision_element = link_tag.find(".collision")
        if collision_element is not None:
            # For the main body, hold the tag to calculate below
            if collision_element.text.strip() == "@base_link_collision":
                base_link_collision_tag = collision_element
            # For these links, calculate volume so they're neutrally buoyant
            # and can be left out of moment computations.
            # * horizontal_fin
            # * vertical_fin
            # * propeller
            # * buoyancy_engine
            elif collision_element.text.strip() == "@neutral_buoyancy":
                mass = link_tag.find(".inertial/mass")
                assert mass is not None, "Mass not found"

                collision_element.text = ""
                geometry = ET.SubElement(collision_element, "geometry")
                box = ET.SubElement(geometry, "box")
                size = ET.SubElement(box, "size")
                dx = 0.1
                dy = 0.1

                # density * gravity * volume = mass * gravity
                # density * dx * dy * dz = mass
                # dz = mass / (density * dx * dy)
                dz = float(mass.text) / (dx * dy * fluid_density)
                size.text = write_float_array([dx, dy, dz])
                continue
            else:
                assert False, "Detected unknown collision"

        # Only these links get here
        link_name = link_tag.get("name")
        assert link_name == "base_link" or link_name == "battery" or link_name == "drop_weight", link_name

        # Get various tags used to calculate moments
        inertial_tag = link_tag.find(".inertial")
        assert inertial_tag is not None, "Missing inertial"

        mass_tag = link_tag.find(".inertial/mass")
        assert mass_tag is not None, "Missing mass"

        link_pose_tag = link_tag.find(".pose")
        inertial_pose_tag = link_tag.find(".inertial/pose")

        # Hold the base link tags to fill below
        if link_name == "base_link":
            base_link_mass_tag = mass_tag
            base_link_inertial_pose_tag = inertial_pose_tag
            continue

        # Get the CoM pose w.r.t. the model origin for each link

        # battery has inertial pose, but no link pose
        if link_name == "battery":
            assert inertial_pose_tag is not None
            assert link_pose_tag is None
            center_of_mass = [float(token) for token in inertial_pose_tag.text.split()]
        # drop_weight has link pose, but no inertial pose
        elif link_name == "drop_weight":
            assert inertial_pose_tag is None
            assert link_pose_tag is not None
            center_of_mass = [float(token) for token in link_pose_tag.text.split()]
        else:
            assert False, "Unknown link"

        assert len(center_of_mass) == 6

        # Moments about the X axis
        x_moments.append((float(mass_tag.text), center_of_mass[0]))

    # Moments and mass for non-neutral links other than the base link (i.e. components)
    component_x_moments =  sum([mass * com for mass, com in x_moments])
    component_mass = sum([mass for mass, _ in x_moments])
    assert component_mass < total_mass, "Component mass is larger than total mass"

    # Calculate the base link's inertial pose and mass
    remaining_mass = total_mass - component_mass
    base_link_inertial_pose_x = - component_x_moments / remaining_mass

    base_link_mass_tag.text = str(remaining_mass)
    base_link_inertial_pose_tag.text = write_float_array([base_link_inertial_pose_x, 0, 0, 0, 0, 0])

    # Calculate base link's volume size and position so that it cancels out the
    # mass of the base link, as well as the components
    base_link_dx = 2.0
    base_link_dy = 0.4
    base_link_dz = total_mass / (2 * 0.4 * fluid_density)

    base_link_collision_tag.text = ""
    pose_tag = ET.SubElement(base_link_collision_tag, "pose")
    pose_tag.text = write_float_array([0, 0, buoyancy_z_offset, 0, 0, 0])

    geometry = ET.SubElement(base_link_collision_tag, "geometry")
    box = ET.SubElement(geometry, "box")
    size = ET.SubElement(box, "size")
    size.text = write_float_array([base_link_dx, base_link_dy, base_link_dz])

    assert component_x_moments + base_link_inertial_pose_x * remaining_mass == 0.0

    # Write to file
    dir_name = path.dirname(output_path)
    if dir_name != '' and not path.exists(dir_name):
        os.makedirs(dir_name)

    sdf_file_input.write(output_path)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage:")
        print("description_generator.py <infile> <outfile>")
        exit(-100)

    generate_model(sys.argv[1], sys.argv[2])
