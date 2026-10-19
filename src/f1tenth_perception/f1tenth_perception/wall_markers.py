"""/viz/walls: every wall track the tracker holds, drawn as a wall.

WHY A SECOND TOPIC AND NOT A WIDER /perception/walls/markers.
/perception/walls/markers draws what the node ASSERTS to the rest of the
stack: wall_tracker.published(), which is at most four ever-confirmed tracks
with a front/left/right/back role, and which deliberately never spends a slot
on 'unknown' (multi_wall_tracker.published()'s own docstring). That topic is
also recorded into every mission bag, so widening it would change the meaning
of a signal past runs are analysed against.

This topic is the other question -- "what does the tracker actually hold right
now" -- and answers it with EVERY track: 'unknown', tentative, coasting and
rejected included, because the walls that are missing from the four slots are
exactly the ones you are looking at Foxglove to understand. It asserts
nothing, nothing consumes it, and it is published only while something is
subscribed.

A TENTATIVE track is drawn as 'unknown' (multi_wall_tracker.reported_role),
grey and labelled so, whatever its geometry says: it has not earned a role
yet. 'unknown' was 'other' until 2026-10-19; ROLE_COLOUR keeps an 'other' key
in the same grey, and role_of reads it (and '') as 'unknown'.

HOW A TRACK IS DRAWN. Four marker slots per track, so one track's markers can
never collide with another's and each can be deleted by id:

    4 * track.id + 0   LINE_STRIP   the segment on the floor, 5 cm wide
    4 * track.id + 1   CUBE         the vertical prism, semi-transparent, floor
                                    to wall_height_m (2.5 m) for every track
    4 * track.id + 2   TEXT         "id role source state dist"
    4 * track.id + 3   LINE_STRIP   the cyan glass outline, glass panes only

Colour is the ROLE and alpha is the STATE, so the two questions never compete
for the same channel. The state word in the text is produced by the same
state_word() the alpha keys on, so a marker cannot say "verified" while being
drawn at the tentative alpha.

THE HEIGHT IS NOT A CHANNEL. Every wall is drawn wall_height_m tall, standing
on the floor, whatever the stereo measured for it -- the drawing only,
Track.z_min / z_max are untouched. Drawing the measured span made the walls of
one corridor a row of different heights in the 3D panel (2.0 m when assumed,
whatever the stereo reported when measured); one fixed height was asked for
(door-detection, 2026-10-19).

COLOURS DIFFER FROM lidar_front_wall_node._ROLE_COLOUR ON PURPOSE-ISH: left is
blue and right is yellow here, green and blue there. That is what was asked
for; it does mean the two wall topics disagree about what blue means, and
output/viz_walls/decisions.md (D4) carries the flag.
"""

import math

from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import wall_fusion

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker, MarkerArray

#: One namespace, so a Foxglove/RViz client can toggle the whole view at once.
NS = 'viz_walls'

SLOT_LINE, SLOT_PRISM, SLOT_TEXT, SLOT_GLASS = 0, 1, 2, 3
SLOTS = (SLOT_LINE, SLOT_PRISM, SLOT_TEXT, SLOT_GLASS)

#: Role -> (r, g, b). 'unknown' is the role for a wall that is a reference for
#: nothing, and every tentative track's; it is the whole point of this topic
#: that it is still drawn. 'other' is its name before 2026-10-19, same colour.
ROLE_COLOUR = {
    'front': (1.00, 0.15, 0.10),   # red: the one that can stop the car
    'left': (0.15, 0.35, 1.00),    # blue
    'right': (1.00, 0.85, 0.10),   # yellow
    'back': (0.60, 0.60, 0.60),    # grey
    'unknown': (0.30, 0.30, 0.30),  # dark neutral grey
    'other': (0.30, 0.30, 0.30),   # legacy name of 'unknown'
}
#: A pane the stereo saw through and the LiDAR ranged: outlined, not filled,
#: because its colour already means its role.
GLASS_COLOUR = (0.10, 0.90, 1.00)

ALPHA = {'verified': 0.9, 'unverified': 0.5, 'rejected': 0.2, 'tentative': 0.2}

LINE_WIDTH_M = 0.05
GLASS_LINE_WIDTH_M = 0.08
#: Every wall's drawn height [m], floor to top (lidar_front_wall_node's
#: wall_viz_height_m): the tracker's own default z_max, so an assumed wall and
#: a measured one look the same, which is the point.
WALL_VIZ_HEIGHT_M = 2.5
PRISM_THICKNESS_M = 0.04
#: The prism is a hint of volume behind the segment, not the thing you read
#: the geometry off, so it is always fainter than its own line.
PRISM_ALPHA_SCALE = 0.45
TEXT_SIZE_M = 0.2
TEXT_CLEARANCE_M = 0.15


def role_of(track):
    """The role the track is drawn with: multi_wall_tracker.reported_role.

    'unknown' for a TENTATIVE track whatever its geometry says, and for the
    empty role (update_roles() has not classified the track yet) and the
    legacy 'other': each of those is a wall that is a reference for nothing,
    which is what 'unknown' means.
    """
    return mwt.reported_role(track)


def state_word(track):
    """One word for the track's standing: tentative, rejected, verified or unverified.

    Merges the two axes a Track carries -- the lifecycle (tentative /
    confirmed / coasting) and the sticky rectangle verdict (unverified /
    verified / rejected) -- in the order a viewer cares about them. Never
    confirmed outranks any verdict: a track that has not earned confirmation
    is tentative whatever the rectangle said about it.
    """
    # The same test reported_role uses, so a track drawn 'unknown' because it
    # is tentative is also SAID to be tentative.
    if mwt.is_tentative(track):
        return 'tentative'
    if track.verify == wall_fusion.VERIFY_REJECTED:
        return 'rejected'
    if track.verify == wall_fusion.VERIFY_VERIFIED:
        return 'verified'
    return 'unverified'


def alpha_of(track):
    """Opacity from state_word, so the alpha and the printed state agree."""
    return ALPHA[state_word(track)]


def source_word(track):
    """'lidar', 'stereo' or 'both' -- what the evidence behind this track is.

    Track.source already ranks both > lidar > stereo and is sticky, so this
    is a read, not a second opinion.
    """
    return wall_fusion.SOURCE_NAMES.get(getattr(track, 'source', None), '?')


def set_lifetime(marker, seconds):
    """Marker.lifetime from a float, split into sec + nanosec correctly.

    nanosec is a uint32 the message class asserts on, so a whole lifetime
    packed into it raises for anything past ~4.295 s -- the assertion that
    once crash-looped mpc_corr on its first corridor publish
    (MPC_corr.py:5355-5368).
    """
    whole = int(seconds)
    marker.lifetime.sec = whole
    marker.lifetime.nanosec = int(round((seconds - whole) * 1e9))
    return marker


class WallMarkerBuilder:
    """Builds /viz/walls arrays and remembers which ids it drew.

    Stateful in exactly one way: the set of track ids in the last array it
    built, so a track that dies gets an explicit DELETE on each of its four
    slots rather than the whole namespace being cleared and rebuilt.
    """

    def __init__(self, lifetime_s, wall_height_m=WALL_VIZ_HEIGHT_M):
        self.lifetime_s = float(lifetime_s)
        #: The prism's height for EVERY track; see the module docstring.
        self.wall_height_m = float(wall_height_m)
        self.drawn_ids = set()

    def forget(self):
        """Drop the memory of what was drawn, without emitting anything.

        Called when the publisher goes quiet because nothing is subscribed:
        the markers expire on their own lifetime, so the next array that is
        actually sent should not open with DELETEs for ids no live client ever
        saw.
        """
        self.drawn_ids = set()

    def build(self, tracks, stamp, frame_id, distances):
        """One MarkerArray for every track in `tracks`.

        `stamp` is the stamp of the scan the tracks were stepped with, not
        now: these markers are a statement about that scan.
        `distances` maps track id -> distance [m] for the label; a missing or
        non-finite entry prints as 'nan'.
        """
        markers = MarkerArray()
        for track in tracks:
            markers.markers.extend(
                self._track_markers(track, stamp, frame_id,
                                    distances.get(track.id, math.nan)))
        alive = {track.id for track in tracks}
        for dead_id in sorted(self.drawn_ids - alive):
            for slot in SLOTS:
                markers.markers.append(self._delete(dead_id, slot, stamp, frame_id))
        self.drawn_ids = alive
        return markers

    def _header(self, marker, stamp, frame_id):
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = NS
        return marker

    def _delete(self, track_id, slot, stamp, frame_id):
        marker = self._header(Marker(), stamp, frame_id)
        marker.id = 4 * track_id + slot
        marker.action = Marker.DELETE
        return marker

    def _track_markers(self, track, stamp, frame_id, distance):
        (x0, y0), (x1, y1) = track.endpoints()
        role = role_of(track)
        r, g, b = ROLE_COLOUR.get(role, ROLE_COLOUR[mwt.ROLE_UNKNOWN])
        alpha = alpha_of(track)
        z_min, z_max = 0.0, self.wall_height_m

        line = self._header(Marker(), stamp, frame_id)
        line.id = 4 * track.id + SLOT_LINE
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = LINE_WIDTH_M
        line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, alpha
        # Just off the floor, so it is not z-fighting the costmap grid.
        line.points = [Point(x=float(x0), y=float(y0), z=0.02),
                       Point(x=float(x1), y=float(y1), z=0.02)]
        set_lifetime(line, self.lifetime_s)

        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        prism = self._header(Marker(), stamp, frame_id)
        prism.id = 4 * track.id + SLOT_PRISM
        prism.type, prism.action = Marker.CUBE, Marker.ADD
        prism.pose.position = Point(x=float(0.5 * (x0 + x1)), y=float(0.5 * (y0 + y1)),
                                    z=float(0.5 * (z_min + z_max)))
        prism.pose.orientation.z = math.sin(yaw / 2.0)
        prism.pose.orientation.w = math.cos(yaw / 2.0)
        prism.scale.x = max(float(length), 0.01)
        prism.scale.y = PRISM_THICKNESS_M
        prism.scale.z = max(float(z_max - z_min), 0.01)
        prism.color.r, prism.color.g, prism.color.b = r, g, b
        prism.color.a = alpha * PRISM_ALPHA_SCALE
        set_lifetime(prism, self.lifetime_s)

        text = self._header(Marker(), stamp, frame_id)
        text.id = 4 * track.id + SLOT_TEXT
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position = Point(x=float(0.5 * (x0 + x1)), y=float(0.5 * (y0 + y1)),
                                   z=float(z_max + TEXT_CLEARANCE_M))
        text.pose.orientation.w = 1.0
        text.scale.z = TEXT_SIZE_M
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 1.0
        # The SOURCE is in the label because a wall the LiDAR has never seen
        # and one both sensors agree on are not the same claim, and on
        # /viz/walls they otherwise look identical (P3).
        text.text = (f'{track.id} {role} {source_word(track)} '
                     f'{state_word(track)} {distance:.2f}m')
        set_lifetime(text, self.lifetime_s)

        out = [line, prism, text]
        if track.glass:
            # Outline, drawn at the top of the pane so it reads as an edge
            # rather than as a second floor line.
            glass = self._header(Marker(), stamp, frame_id)
            glass.id = 4 * track.id + SLOT_GLASS
            glass.type, glass.action = Marker.LINE_STRIP, Marker.ADD
            glass.pose.orientation.w = 1.0
            glass.scale.x = GLASS_LINE_WIDTH_M
            gr, gg, gb = GLASS_COLOUR
            glass.color.r, glass.color.g, glass.color.b, glass.color.a = gr, gg, gb, 0.9
            glass.points = [Point(x=float(x0), y=float(y0), z=float(z_max)),
                            Point(x=float(x1), y=float(y1), z=float(z_max))]
            set_lifetime(glass, self.lifetime_s)
            out.append(glass)
        else:
            # A track that stopped being glass must lose its outline, and the
            # outline is a marker of its own, so it needs its own DELETE.
            out.append(self._delete(track.id, SLOT_GLASS, stamp, frame_id))
        return out
