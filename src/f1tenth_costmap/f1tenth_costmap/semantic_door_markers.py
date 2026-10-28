"""/viz/doors_semantic: the semantic-layer doors, drawn for Foxglove.

Published by semantic_layer_node on every LiDAR cycle while the semantic door
detector is on, subscribed or not (so the mission logger records it and a
recorded run shows doors too), in the MAP frame. Nothing consumes it.

HOW A DOOR IS DRAWN. Five marker slots per door, so one door's markers never
collide with another's and each can be deleted by id:

    5 * id + 0   CYLINDER     jamb0: diameter jamb_diameter_m, height_m tall,
    5 * id + 1   CYLINDER     jamb1  standing on the floor (z = 0)
    5 * id + 2   LINE_STRIP   the threshold, jamb to jamb, on the floor
    5 * id + 3   ARROW        the CROSSING DIRECTION: the door frame's x axis,
                              from the centre outward. This is the one thing
                              worth looking at in Foxglove before trusting an
                              enter_door mission -- it is fixed at first
                              confirmation and must not flip after the car
                              drives through.
    5 * id + 4   TEXT         "door <id> <width>m <case> <state>"

Colour is the CASE: case A (two collinear walls) green, case B (a wall end
against a perpendicular wall) blue, case F (a scan-jump opening, from
freespace_doors.py -- no walls at all) amber, so a glance says which rule
found it.
Alpha is HOW MUCH IT HAS EARNED: STRONG solid, merely CONFIRMED half-lit,
candidate faint. Only a strong door can be selected by an enter_door mission
(semantic_doors.DoorTracker.is_strong), so the difference between solid and
half-lit is the difference between a door the car will drive at and one it
will refuse -- which is worth seeing without reading a topic.

A door that dies gets a DELETE on each of its five slots, and every marker
carries a lifetime, so a stalled node fades out instead of leaving ghosts --
the same convention /viz/doors and /perception/walls/markers already follow,
and for the same reason (no DELETEALL, which flickers).
"""

import math

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker, MarkerArray

NS = 'viz_doors_semantic'

SLOTS = 5
SLOT_JAMB0, SLOT_JAMB1, SLOT_LINE, SLOT_ARROW, SLOT_TEXT = range(SLOTS)

CASE_A_COLOUR = (0.10, 0.90, 0.40)   # green: two collinear walls
CASE_B_COLOUR = (0.20, 0.55, 1.00)   # blue: a wall end against a perpendicular
CASE_F_COLOUR = (1.00, 0.65, 0.10)   # amber: a free-space bottleneck, no walls
CASE_COLOURS = {'A': CASE_A_COLOUR, 'B': CASE_B_COLOUR, 'F': CASE_F_COLOUR}
#: Three tiers, because a door has three states worth telling apart at a
#: glance: STRONG (a mission may drive at it) solid, merely CONFIRMED
#: (published, but enter_door will not select it) half-lit, and a CANDIDATE
#: faint. See semantic_doors.DoorTracker.is_strong.
CANDIDATE_ALPHA = 0.30
CONFIRMED_ALPHA = 0.60
STRONG_ALPHA = 1.0

LINE_WIDTH_M = 0.03
ARROW_LEN_M = 0.6
ARROW_SHAFT_M = 0.04
ARROW_HEAD_M = 0.10
TEXT_SIZE_M = 0.2
TEXT_CLEARANCE_M = 0.2


def set_lifetime(marker, seconds):
    """Set Marker.lifetime from a float, split into sec + nanosec correctly."""
    whole = int(seconds)
    marker.lifetime.sec = whole
    marker.lifetime.nanosec = int(round((seconds - whole) * 1e9))
    return marker


def colour_of(door):
    """(r, g, b, a): the case picks the hue, the state the alpha.

    An unknown case falls back to case B's blue rather than raising -- this is
    a drawing function and a new door rule must never be able to stop the
    markers from going out.
    """
    r, g, b = CASE_COLOURS.get(door.case, CASE_B_COLOUR)
    if not door.confirmed:
        return r, g, b, CANDIDATE_ALPHA
    return r, g, b, (STRONG_ALPHA if getattr(door, 'strong', True)
                     else CONFIRMED_ALPHA)


def label_of(door):
    """The text a door is labelled with: id, width, case, strength, flags.

    The flags are the half of a door's story the geometry does not tell --
    `marginal` (width outside the nominal band), `one_jamb` (only one end had
    occupancy behind it), `no_grid` (never cross-checked against a map). All
    three are reasons to look at a door twice before driving at it, and none
    of them is visible from the markers alone.
    """
    flags = getattr(door, 'flags', ()) or ()
    tail = (' [' + ' '.join(flags) + ']') if flags else ''
    if not door.confirmed:
        state = 'candidate'
    else:
        state = 'STRONG' if getattr(door, 'strong', True) else 'confirmed'
    return (f'door {door.id} {door.width:.2f}m case {door.case} {state}{tail}')


def _base(door, frame_id, stamp, slot, kind, lifetime):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = NS
    m.id = SLOTS * int(door.id) + slot
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.r, m.color.g, m.color.b, m.color.a = colour_of(door)
    return set_lifetime(m, lifetime)


def door_markers(door, frame_id, stamp, jamb_diameter_m=0.10, height_m=1.0,
                 lifetime=1.0):
    """The five markers of one door."""
    j0, j1 = door.jambs()
    out = []
    for slot, (jx, jy) in ((SLOT_JAMB0, j0), (SLOT_JAMB1, j1)):
        m = _base(door, frame_id, stamp, slot, Marker.CYLINDER, lifetime)
        m.pose.position.x, m.pose.position.y = float(jx), float(jy)
        m.pose.position.z = height_m / 2.0
        m.scale.x = m.scale.y = float(jamb_diameter_m)
        m.scale.z = float(height_m)
        out.append(m)

    line = _base(door, frame_id, stamp, SLOT_LINE, Marker.LINE_STRIP, lifetime)
    line.scale.x = LINE_WIDTH_M
    line.points = [Point(x=float(j0[0]), y=float(j0[1]), z=0.0),
                   Point(x=float(j1[0]), y=float(j1[1]), z=0.0)]
    out.append(line)

    n = door.normal or (-door.axis[1], door.axis[0])
    arrow = _base(door, frame_id, stamp, SLOT_ARROW, Marker.ARROW, lifetime)
    arrow.scale.x, arrow.scale.y, arrow.scale.z = (
        ARROW_SHAFT_M, ARROW_HEAD_M, ARROW_HEAD_M)
    arrow.points = [
        Point(x=float(door.centre[0]), y=float(door.centre[1]), z=0.02),
        Point(x=float(door.centre[0] + n[0] * ARROW_LEN_M),
              y=float(door.centre[1] + n[1] * ARROW_LEN_M), z=0.02)]
    out.append(arrow)

    text = _base(door, frame_id, stamp, SLOT_TEXT, Marker.TEXT_VIEW_FACING, lifetime)
    text.pose.position.x, text.pose.position.y = (float(door.centre[0]),
                                                  float(door.centre[1]))
    text.pose.position.z = height_m + TEXT_CLEARANCE_M
    text.scale.z = TEXT_SIZE_M
    text.text = label_of(door)
    out.append(text)
    return out


def delete_markers(door_id, frame_id, stamp):
    """A DELETE on each of a dead door's five slots."""
    out = []
    for slot in range(SLOTS):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = NS
        m.id = SLOTS * int(door_id) + slot
        m.action = Marker.DELETE
        out.append(m)
    return out


def marker_array(doors, previous_ids, frame_id, stamp, jamb_diameter_m=0.10,
                 height_m=1.0, lifetime=1.0):
    """(MarkerArray, ids drawn): every live door, plus DELETEs for the gone ones.

    `doors` is every live track, candidates included -- /viz always draws both,
    whatever /perception/doors_semantic carries.
    """
    msg = MarkerArray()
    live = set()
    for door in doors:
        live.add(int(door.id))
        msg.markers.extend(door_markers(
            door, frame_id, stamp, jamb_diameter_m, height_m, lifetime))
    for gone in sorted(set(previous_ids) - live):
        msg.markers.extend(delete_markers(gone, frame_id, stamp))
    return msg, live


def door_message(door, Door, stamp_msg):
    """One f1tenth_messages/Door from a DoorTrack, MAP frame.

    THE MESSAGE IS SHARED WITH THE PROFILE DETECTOR deliberately: a reader of
    a door should not have to know which detector found it. The fields that
    only a beam profile can fill (leaf_side, n_beams, stereo_support,
    glass_suspect, occupied) are left at their defaults, and
    jamb_height_unknown is TRUE because nothing here ever measured a jamb's
    height. The *_bl copies are NOT filled: this detector works in the map
    frame and has no scan-stamp base_link pose to express them at, where the
    profile detector works in base_link and gets them for free. A consumer
    that needs base_link transforms the map coordinates itself, through the
    same map -> base_link edge it would need anyway.
    """
    m = Door()
    m.id = int(door.id)
    m.state = Door.STATE_CONFIRMED if door.confirmed else Door.STATE_CANDIDATE
    j0, j1 = door.jambs()
    m.center.x, m.center.y = float(door.centre[0]), float(door.centre[1])
    m.jamb0.x, m.jamb0.y = float(j0[0]), float(j0[1])
    m.jamb1.x, m.jamb1.y = float(j1[0]), float(j1[1])
    n = door.normal or (-door.axis[1], door.axis[0])
    m.normal.x, m.normal.y = float(n[0]), float(n[1])
    m.width = float(door.width)
    m.parent_wall_id0 = int(door.wall_id0)
    m.parent_wall_id1 = int(door.wall_id1)
    m.hits = int(door.hits)
    m.confidence = 0.8 if door.confirmed else 0.5
    m.jamb_height_unknown = True
    # What a mission is allowed to drive at -- see Door.msg's own comment and
    # semantic_doors.DoorTracker.is_strong. getattr, not attribute access,
    # because the profile detector builds its own Door messages from its own
    # track type, which has no such attribute and wants the old behaviour
    # (every confirmed door selectable).
    m.strong = bool(getattr(door, 'strong', True))
    stamp_sec = stamp_msg.sec + stamp_msg.nanosec * 1e-9
    m.age = float(max(0.0, stamp_sec - door.born_sec))
    m.since_seen = float(max(0.0, stamp_sec - door.last_seen_sec))
    return m


def door_object(door, SemanticObject):
    """One SemanticObject of class 'door' for /costmap/semantic_doors.

    radius_m is the door's HALF WIDTH: the disk that just covers the opening.
    It is a footprint for drawing and for "is the car at the door", never an
    obstacle -- which is exactly why these go on a topic of their own and not
    on /costmap/semantic_objects (see stack_params, door_source).
    """
    o = SemanticObject()
    o.id = int(door.id)
    o.class_id = 'door'
    o.x, o.y = float(door.centre[0]), float(door.centre[1])
    o.radius_m = float(door.width / 2.0)
    o.effective_radius_m = o.radius_m
    o.state = SemanticObject.TRACKED
    o.confidence = 1.0
    o.score = 1.0
    o.in_fov = False
    o.in_fov_lidar = False
    o.in_fov_stereo = False
    return o


def door_transform(door, TransformStamped, parent_frame, stamp_msg):
    """map -> door_<id>: the origin at the jamb midpoint, x the crossing direction."""
    t = TransformStamped()
    t.header.stamp = stamp_msg
    t.header.frame_id = parent_frame
    t.child_frame_id = door.frame_id()
    t.transform.translation.x = float(door.centre[0])
    t.transform.translation.y = float(door.centre[1])
    t.transform.translation.z = 0.0
    yaw = door.yaw()
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


# ---------------------------------------------------------------------------
# /viz/doors_freespace_debug: what freespace_doors.py saw, and what it threw
# away. A SEPARATE namespace and a separate topic from the doors above,
# because this is the detector's own working-out, not a door: the medial axis
# of the free space and every bottleneck on it, the rejected ones included.
# Nothing consumes it; it exists so that "why is there no door there" is one
# Foxglove panel rather than a replay.
# ---------------------------------------------------------------------------
DEBUG_NS = 'viz_doors_freespace'

DEBUG_SLOT_JUMPS = 0
DEBUG_SLOT_CANDIDATES = 1
DEBUG_SLOT_REJECTED = 2
DEBUG_SLOT_SKELETON = 3
DEBUG_SLOT_LABELS = 100          # + i, one per candidate

#: Yellow: every range discontinuity the scan half found, before any pairing.
#: This is the layer to look at when a door is missing entirely -- if there
#: is no yellow at the jambs, the opening never produced a near-far-near step
#: and no amount of parameter tuning further down will help.
DEBUG_JUMP_COLOUR = (1.00, 0.95, 0.20, 0.85)
#: Orange: an opening that passed the scan half. Bright when it also passed
#: the grid cross-check, dim when the grid rejected it.
DEBUG_CANDIDATE_COLOUR = (1.00, 0.55, 0.10, 0.95)
DEBUG_REJECTED_COLOUR = (0.55, 0.55, 0.55, 0.60)
#: Blue: the medial axis, drawn only when use_skeleton is on.
DEBUG_SKELETON_COLOUR = (0.35, 0.75, 1.00, 0.55)

DEBUG_JUMP_SIZE_M = 0.07
DEBUG_CANDIDATE_SIZE_M = 0.12
DEBUG_SKELETON_SIZE_M = 0.03
DEBUG_LABEL_SIZE_M = 0.10


def _debug_base(frame_id, stamp, slot, kind, lifetime):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = DEBUG_NS
    m.id = int(slot)
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return set_lifetime(m, lifetime)


def _point_cloud(frame_id, stamp, slot, kind, xy, size, colour, lifetime, z=0.02):
    m = _debug_base(frame_id, stamp, slot, kind, lifetime)
    m.scale.x = m.scale.y = m.scale.z = float(size)
    m.color.r, m.color.g, m.color.b, m.color.a = colour
    # `xy if xy is not None` and NOT `xy or ()`: these arrive as numpy
    # arrays, and the truth value of an array with more than one element is
    # a ValueError -- which, caught by the node's own cycle guard, showed up
    # as "debug markers silently never published" rather than as a crash.
    rows = () if xy is None else xy
    m.points = [Point(x=float(x), y=float(y), z=z) for x, y in rows]
    return m


def freespace_debug_markers(jump_xy, candidates, frame_id, stamp,
                            skeleton_xy=None, lifetime=2.0, max_labels=16):
    """What the detector saw and what it threw away, in three layers.

    `jump_xy` is every range discontinuity (freespace_doors.jump_points),
    `candidates` the list freespace_doors.scan_candidates returned --
    accepted AND rejected, which is the point -- and `skeleton_xy` the medial
    axis, only when the optional skeleton detector is running.

    FIXED MARKER IDS with always-published (possibly empty) point lists: the
    counts change every cycle, and per-candidate ids would need DELETEs
    chasing them. A layer that stops matching anything visibly empties
    instead of freezing on screen. The code labels do get one id each, capped
    at `max_labels`, and the unused ones are deleted.
    """
    msg = MarkerArray()
    msg.markers.append(_point_cloud(
        frame_id, stamp, DEBUG_SLOT_JUMPS, Marker.POINTS, jump_xy,
        DEBUG_JUMP_SIZE_M, DEBUG_JUMP_COLOUR, lifetime, z=0.01))

    accepted = [c for c in candidates if c.code == '']
    rejected = [c for c in candidates if c.code != '']
    msg.markers.append(_point_cloud(
        frame_id, stamp, DEBUG_SLOT_CANDIDATES, Marker.CUBE_LIST,
        [c.centre for c in accepted], DEBUG_CANDIDATE_SIZE_M,
        DEBUG_CANDIDATE_COLOUR, lifetime, z=0.05))
    msg.markers.append(_point_cloud(
        frame_id, stamp, DEBUG_SLOT_REJECTED, Marker.CUBE_LIST,
        [c.centre for c in rejected], DEBUG_CANDIDATE_SIZE_M,
        DEBUG_REJECTED_COLOUR, lifetime, z=0.05))
    msg.markers.append(_point_cloud(
        frame_id, stamp, DEBUG_SLOT_SKELETON, Marker.POINTS, skeleton_xy,
        DEBUG_SKELETON_SIZE_M, DEBUG_SKELETON_COLOUR, lifetime, z=0.01))

    ordered = accepted + rejected
    for i in range(int(max_labels)):
        text = _debug_base(frame_id, stamp, DEBUG_SLOT_LABELS + i,
                           Marker.TEXT_VIEW_FACING, lifetime)
        if i >= len(ordered):
            text.action = Marker.DELETE
            msg.markers.append(text)
            continue
        cand = ordered[i]
        text.pose.position.x = float(cand.centre[0])
        text.pose.position.y = float(cand.centre[1])
        text.pose.position.z = 0.30
        text.scale.z = DEBUG_LABEL_SIZE_M
        colour = DEBUG_CANDIDATE_COLOUR if cand.code == '' else DEBUG_REJECTED_COLOUR
        text.color.r, text.color.g, text.color.b, text.color.a = colour
        label = cand.code or 'door'
        if cand.flags:
            label += ' [' + ' '.join(cand.flags) + ']'
        text.text = f'{label} {cand.width:.2f}m'
        msg.markers.append(text)
    return msg


#: What a door detector publishes, for the whitelists and the docs. Shared by
#: all three sources (see semantic_doors.DOOR_SOURCES): at most one of them
#: runs, so these names say "the stack's doors", not "the semantic ones".
TOPICS = ('/perception/doors_semantic', '/viz/doors_semantic',
          '/costmap/semantic_doors')

#: freespace_door_node's own two extras, which no mission or controller reads.
FREESPACE_DEBUG_TOPICS = ('/viz/doors_freespace_debug',
                          '/costmap/freespace_doors/debug')
