#!/usr/bin/env python3

"""
Build Gazebo world and fiducial_vlam map files from a list of markers and poses
Usage:
    cd src/sim_fiducial/worlds
    python3 build_world.py

Transformation notation:
   t_destination_source is a transform
   vector_destination = t_destination_source * vector_source
   xxx_f_destination means xxx is expressed in destination frame
   xxx_pose_f_destination is equivalent to t_destination_xxx
   t_a_c = t_a_b * t_b_c

Also:
   r_destination_source is a fixed axis rotation, i.e.,
   roll, pitch, yaw about X, Y, Z, per https://www.ros.org/reps/rep-0103.html
"""

import math
import transformations as xf

# SDF and fiducial_vlam have different coordinate models
t_world_map = xf.quaternion_matrix([math.sqrt(0.5), 0, 0, -math.sqrt(0.5)])


def build_world(name, markers):
    world_file = open(name, 'w')
    world_file.write("""<?xml version="1.0"?>

<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://sun</uri>
    </include>

""")
    for marker in markers:
        world_file.write(f"""    <model name="marker{marker[0]}">
      <include>
        <static>true</static>
        <uri>model://marker_{marker[0]}</uri>
      </include>
      <pose>{marker[1]} {marker[2]} {marker[3]} {marker[4]} {marker[5]} {marker[6]}</pose>
    </model>
""")
    world_file.write("""  </world>
</sdf>""")
    world_file.close()


def build_map(name, markers):
    map_file = open(name, 'w')
    map_file.write("""# All marker locations are fixed (f: 1)

marker_length: 0.1778
markers:
""")
    for marker in markers:
        # axes='sxyz' is the default in transformations.py, but make it explicit for clarity
        t_marker_world = xf.euler_matrix(marker[4], marker[5], marker[6], axes='sxyz')
        t_marker_map = t_marker_world @ t_world_map
        r_marker_map = xf.euler_from_matrix(t_marker_map, axes='sxyz')

        map_file.write(f"""  - id: {marker[0]}
    u: 1
    f: 1
    xyz: [{marker[1]}, {marker[2]}, {marker[3]}]
    rpy: [{r_marker_map[0]}, {r_marker_map[1]}, {r_marker_map[2]}]
""")
    map_file.close()


def vertical_plane_of_markers(id_first, rows, cols, spacing, origin_pose):
    xforms = [origin_pose @ xf.compose_matrix(None, None, [0, -math.pi / 2, 0],
                                              [0, (c - (cols - 1) / 2) * spacing, r * spacing])
              for r in range(rows)
              for c in range(cols)]
    decompositions = [xf.decompose_matrix(xform) for xform in xforms]
    return [[id_first + idx, d[3][0], d[3][1], d[3][2], d[2][0], d[2][1], d[2][2]]
            for idx, d in enumerate(decompositions)]


def circle_of_planes_of_markers(id_first, rows, cols, spacing, radius, n):
    rotated_planes = []
    delta_theta = math.pi * 2. / n
    for i in range(n):
        theta = delta_theta * i
        rotated_planes += vertical_plane_of_markers(
            i * rows * cols + id_first, rows, cols, spacing,
            xf.compose_matrix(None, None, [0., 0., theta],
                              [radius * math.cos(theta), radius * math.sin(theta), 0.], None))
    return rotated_planes


# A world with a single marker
# Marker format: [marker_num, x, y, z, roll, pitch, yaw]
one_marker = [
    [0, 2, 0, 0, 0, -math.pi / 2, 0],
]

# A world with 8 markers in the yz plane
yz_plane_of_markers = vertical_plane_of_markers(0, 2, 4, 1., xf.compose_matrix(None, None, None, [2., 0., 0.], None))

# A box of markers
box_of_markers = circle_of_planes_of_markers(0, 2, 1, 1., 2., 4)

# a circle of markers
circle_of_markers = circle_of_planes_of_markers(0, 1, 1, 1., 2., 8)

worlds = [
    ['one_marker.world', 'one_marker_map.yaml', one_marker],
    ['yz_plane_of_markers.world', 'yz_plane_of_markers_map.yaml', yz_plane_of_markers],
    ['box_of_markers.world', 'box_of_markers_map.yaml', box_of_markers],
    ['circle_of_markers.world', 'circle_of_markers_map.yaml', circle_of_markers],
]

for world in worlds:
    build_world(world[0], world[2])
    build_map(world[1], world[2])
