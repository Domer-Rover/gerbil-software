#!/usr/bin/env python3
"""Generate worlds/school_loop.sdf: a rectangular ring corridor around a solid classroom block.

The layout is the school hallway Gerbil drives in: a corridor that wraps around a rectangle, 12 ft
(3.66 m) wide, with the classrooms in the middle treated as one solid block. Door recesses are cut
into the classroom (inner) walls. They only widen the corridor, so the clear width is never less
than --corridor.

Coordinates: the world origin is the centre of the block, x runs along the long side and y along
the short side. The outer dimensions are the clear inside faces of the outer walls.

Dependency-free (Python 3 standard library only). Run with no arguments to regenerate the
committed SDF exactly:

    python3 src/gerbil_gazebo/scripts/gen_loop_world.py
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, '..', 'worlds', 'school_loop.sdf'))

WALL_THICKNESS = 0.2      # outer walls, m
DOOR_WIDTH = 1.0          # m
DOOR_DEPTH = 0.25         # how far a door recess sets back into the classroom block, m
DOOR_HEIGHT = 2.1         # m
CORNER_CLEARANCE = 1.0    # keep doors this far from a block corner, m

FLOOR_RGB = (0.86, 0.84, 0.78)   # pale terrazzo
OUTER_RGB = (0.30, 0.40, 0.55)   # slate blue
INNER_RGB = (0.62, 0.32, 0.24)   # brick
DOOR_RGB = (0.42, 0.27, 0.13)    # wood
LINE_RGB = (0.95, 0.75, 0.05)    # start/finish line


def f(v):
    """Format a number compactly and deterministically."""
    v = round(v, 4)
    if v == 0:
        v = 0.0  # no "-0"
    s = ('%.4f' % v).rstrip('0').rstrip('.')
    return s


def vec(*vals):
    return ' '.join(f(v) for v in vals)


def material(rgb):
    r, g, b = rgb
    return (
        '<material>'
        '<ambient>%s 1</ambient><diffuse>%s 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
        '</material>' % (vec(r, g, b), vec(r, g, b))
    )


def box_element(name, x, y, z, sx, sy, sz, rgb, collision=True, indent='        '):
    """One <collision>/<visual> pair (or visual only) for a box centred at (x, y, z)."""
    pose = vec(x, y, z, 0, 0, 0)
    size = vec(sx, sy, sz)
    out = []
    if collision:
        out.append('%s<collision name="%s_col">' % (indent, name))
        out.append('%s  <pose>%s</pose>' % (indent, pose))
        out.append('%s  <geometry><box><size>%s</size></box></geometry>' % (indent, size))
        out.append('%s</collision>' % indent)
    out.append('%s<visual name="%s_vis">' % (indent, name))
    out.append('%s  <pose>%s</pose>' % (indent, pose))
    out.append('%s  <geometry><box><size>%s</size></box></geometry>' % (indent, size))
    out.append('%s  %s' % (indent, material(rgb)))
    out.append('%s</visual>' % indent)
    return out


def allocate_doors(n, lengths):
    """Spread n doors over the four sides in proportion to side length (largest remainder)."""
    total = sum(lengths)
    shares = [n * l / total for l in lengths]
    counts = [int(math.floor(s)) for s in shares]
    order = sorted(range(len(lengths)), key=lambda i: (-(shares[i] - counts[i]), i))
    for i in order[: n - sum(counts)]:
        counts[i] += 1
    return counts


def split_span(lo, hi, centres, width):
    """Segments of [lo, hi] left after cutting out door openings of `width` at `centres`."""
    segs = []
    cur = lo
    for c in sorted(centres):
        a, b = c - width / 2.0, c + width / 2.0
        if a > cur + 1e-6:
            segs.append((cur, a))
        cur = b
    if hi > cur + 1e-6:
        segs.append((cur, hi))
    return segs


def build(args):
    L, W, c, h = args.outer_length, args.outer_width, args.corridor, args.wall_height
    t = WALL_THICKNESS
    a, b = L / 2.0 - c, W / 2.0 - c          # half extents of the classroom block
    d = DOOR_DEPTH
    if a - d <= 0.5 or b - d <= 0.5:
        sys.exit('error: corridor too wide for the outer size; the classroom block vanishes')

    centreline = 2.0 * ((L - c) + (W - c))
    spawn_x, spawn_y, spawn_yaw = 0.0, -(W / 2.0 - c / 2.0), 0.0

    # Door positions. Sides: south (y=-b), north (y=+b) run along x; east (x=+a), west (x=-a)
    # run along y. Each side's usable span stays CORNER_CLEARANCE clear of the corners.
    side_len = [2 * a, 2 * a, 2 * b, 2 * b]
    counts = allocate_doors(args.doors, side_len) if args.doors > 0 else [0, 0, 0, 0]
    doors = []  # (side, centre along the side)
    for side, (n, length) in enumerate(zip(counts, side_len)):
        half = length / 2.0
        usable = length - 2 * CORNER_CLEARANCE
        if n and n * (DOOR_WIDTH + 0.5) > usable:
            sys.exit('error: too many doors (%d) for a %.2f m wall' % (n, length))
        for k in range(n):
            doors.append((side, -half + CORNER_CLEARANCE + usable * (k + 1) / (n + 1)))

    walls = []
    # Outer walls: their inner faces sit exactly on the outer dimensions.
    walls += box_element('outer_south', 0, -(W / 2 + t / 2), h / 2, L + 2 * t, t, h, OUTER_RGB)
    walls += box_element('outer_north', 0, W / 2 + t / 2, h / 2, L + 2 * t, t, h, OUTER_RGB)
    walls += box_element('outer_east', L / 2 + t / 2, 0, h / 2, t, W, h, OUTER_RGB)
    walls += box_element('outer_west', -(L / 2 + t / 2), 0, h / 2, t, W, h, OUTER_RGB)

    # Classroom block: a core inset by the recess depth, wrapped in a skin of depth d that has
    # the door openings cut out of it. North/south skins span the full length (they own the
    # corners); east/west skins fit between them.
    walls += box_element('block_core', 0, 0, h / 2, 2 * (a - d), 2 * (b - d), h, INNER_RGB)
    for side in range(4):
        centres = [p for s, p in doors if s == side]
        if side < 2:
            ysign = -1 if side == 0 else 1
            for i, (lo, hi) in enumerate(split_span(-a, a, centres, DOOR_WIDTH)):
                walls += box_element('block_%s_%d' % ('south' if side == 0 else 'north', i),
                                     (lo + hi) / 2, ysign * (b - d / 2), h / 2,
                                     hi - lo, d, h, INNER_RGB)
        else:
            xsign = 1 if side == 2 else -1
            for i, (lo, hi) in enumerate(split_span(-(b - d), b - d, centres, DOOR_WIDTH)):
                walls += box_element('block_%s_%d' % ('east' if side == 2 else 'west', i),
                                     xsign * (a - d / 2), (lo + hi) / 2, h / 2,
                                     d, hi - lo, h, INNER_RGB)

    # Door leaves: visual only, standing 1 cm proud of the core face at the back of each recess.
    dt = 0.04
    for i, (side, p) in enumerate(doors):
        if side == 0:
            x, y, sx, sy = p, -(b - d) - dt / 2 + 0.01, DOOR_WIDTH - 0.1, dt
        elif side == 1:
            x, y, sx, sy = p, (b - d) + dt / 2 - 0.01, DOOR_WIDTH - 0.1, dt
        elif side == 2:
            x, y, sx, sy = (a - d) + dt / 2 - 0.01, p, dt, DOOR_WIDTH - 0.1
        else:
            x, y, sx, sy = -(a - d) - dt / 2 + 0.01, p, dt, DOOR_WIDTH - 0.1
        walls += box_element('door_%d' % i, x, y, DOOR_HEIGHT / 2, sx, sy, DOOR_HEIGHT, DOOR_RGB,
                             collision=False)

    # Start/finish line across the spawn corridor (visual only, 2 mm high).
    walls += box_element('start_line', spawn_x, spawn_y, 0.001, 0.1, c, 0.002, LINE_RGB,
                         collision=False)

    step = args.step
    lines = []
    w = lines.append
    w('<?xml version="1.0"?>')
    w('<!--')
    w('  school_loop: rectangular ring corridor around a solid classroom block.')
    w('  GENERATED by scripts/gen_loop_world.py - edit the script, not this file.')
    # No double hyphens: they are illegal inside XML comments, so no "--option" spellings here.
    w('  Args: outer-length %s, outer-width %s, corridor %s, wall-height %s, doors %d, step %s'
      % (f(L), f(W), f(c), f(h), args.doors, f(step)))
    w('')
    w('  Outer (clear) size %s x %s m, corridor %s m clear everywhere (door recesses only widen it),'
      % (f(L), f(W), f(c)))
    w('  classroom block %s x %s m, walls %s m high.' % (f(2 * a), f(2 * b), f(h)))
    w('  LAP LENGTH along the corridor centreline: %s m (%s x %s m rectangle).'
      % (f(centreline), f(L - c), f(W - c)))
    w('  Spawn (config/worlds.yaml): x=%s y=%s yaw=%s, mid south corridor facing +x;'
      % (f(spawn_x), f(spawn_y), f(spawn_yaw)))
    w('  driving forward laps counter-clockwise. The yellow line on the floor marks the start.')
    w('')
    w('  Physics: %s ms steps at real_time_factor 1.0 (%d Hz). ros2_control runs at 50 Hz, i.e.'
      % (f(step * 1000), int(round(1.0 / step))))
    w('  exactly one controller update every %d physics steps, and the 10 Hz LiDAR and 25 Hz'
      % int(round(0.02 / step)))
    w('  odometry also land on whole steps. 4 ms is stable for a small diff-drive base on DART')
    w('  and costs half the CPU of 2 ms, which matters with software rendering eating the rest.')
    w('  If the wheels chatter, regenerate with a 0.002 s step (the step option).')
    w('  Rendering: one directional light, no shadows, no sky, primitive boxes only.')
    w('-->')
    w('<sdf version="1.8">')
    w('  <world name="school_loop">')
    w('    <physics name="loop_physics" type="ignored">')
    w('      <max_step_size>%s</max_step_size>' % f(step))
    w('      <real_time_factor>1.0</real_time_factor>')
    w('      <real_time_update_rate>%d</real_time_update_rate>' % int(round(1.0 / step)))
    # DART's default FCL collision checker gives a cylinder resting on a plane
    # one unstable contact point, and Gerbil's wheels slipped ~87 percent
    # (wheel odom 0.20 m/s, true 0.025 m/s). ODE's checker handles it.
    w('      <dart><collision_detector>ode</collision_detector></dart>')
    w('    </physics>')
    w('')
    w('    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>')
    w('    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>')
    w('    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>')
    w('    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">')
    w('      <render_engine>ogre2</render_engine>')
    w('    </plugin>')
    w('')
    w('    <gravity>0 0 -9.81</gravity>')
    w('    <scene>')
    w('      <ambient>0.55 0.55 0.55 1</ambient>')
    w('      <background>0.75 0.8 0.85 1</background>')
    w('      <shadows>false</shadows>')
    w('      <grid>false</grid>')
    w('    </scene>')
    w('')
    w('    <light type="directional" name="sun">')
    w('      <cast_shadows>false</cast_shadows>')
    w('      <pose>0 0 20 0 0 0</pose>')
    w('      <diffuse>0.85 0.85 0.85 1</diffuse>')
    w('      <specular>0.1 0.1 0.1 1</specular>')
    w('      <direction>-0.4 0.3 -0.85</direction>')
    w('    </light>')
    w('')
    w('    <model name="ground_plane">')
    w('      <static>true</static>')
    w('      <link name="link">')
    w('        <collision name="collision">')
    w('          <geometry><plane><normal>0 0 1</normal><size>%s</size></plane></geometry>'
      % vec(L + 10, W + 10))
    w('          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>')
    w('        </collision>')
    w('        <visual name="visual">')
    w('          <geometry><plane><normal>0 0 1</normal><size>%s</size></plane></geometry>'
      % vec(L + 10, W + 10))
    w('          %s' % material(FLOOR_RGB))
    w('        </visual>')
    w('      </link>')
    w('    </model>')
    w('')
    w('    <model name="school_walls">')
    w('      <static>true</static>')
    w('      <link name="link">')
    lines.extend(walls)
    w('      </link>')
    w('    </model>')
    w('  </world>')
    w('</sdf>')
    return '\n'.join(lines) + '\n', centreline, (spawn_x, spawn_y, spawn_yaw)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--outer-length', type=float, default=40.0, help='outer clear length along x, m')
    p.add_argument('--outer-width', type=float, default=25.0, help='outer clear width along y, m')
    p.add_argument('--corridor', type=float, default=3.66, help='corridor clear width, m (12 ft)')
    p.add_argument('--wall-height', type=float, default=2.4, help='wall height, m')
    p.add_argument('--doors', type=int, default=6, help='door recesses on the classroom walls')
    p.add_argument('--step', type=float, default=0.004, help='physics max_step_size, s')
    p.add_argument('-o', '--output', default=DEFAULT_OUT, help='output SDF (default: worlds/school_loop.sdf)')
    args = p.parse_args()
    if args.outer_width > args.outer_length:
        sys.exit('error: --outer-width must not exceed --outer-length (x is the long side)')
    if min(args.outer_length, args.outer_width, args.corridor, args.wall_height, args.step) <= 0:
        sys.exit('error: sizes and step must be positive')
    sdf, lap, spawn = build(args)
    with open(args.output, 'w') as fh:
        fh.write(sdf)
    print('wrote %s' % args.output)
    print('lap (centreline) %.2f m; spawn x=%s y=%s yaw=%s' % (lap, f(spawn[0]), f(spawn[1]), f(spawn[2])))


if __name__ == '__main__':
    main()
