"""
/viz/doors: every door track door_detection holds, drawn for Foxglove.

Published by lidar_front_wall_node on EVERY wall tracker cycle while door
detection is on, subscribed or not (unlike /viz/walls), so the mission logger
records it and recorded runs show doors in Foxglove too; in the tracker's
fixed frame. '^/viz/.*' in foxglove_topic_whitelist carries it across the
bridge. Nothing consumes it.

HOW A DOOR IS DRAWN. Four marker slots per door, so one door's markers never
collide with another's and each can be deleted by id:

    4 * door.id + 0   CYLINDER     jamb0: diameter jamb_diameter_m, height_m tall,
    4 * door.id + 1   CYLINDER     jamb1  standing on the floor (z = 0)
    4 * door.id + 2   LINE_STRIP   the threshold, jamb to jamb, on the floor
    4 * door.id + 3   TEXT         "door <id> <width>m <state>[ leaf <side>]
                                    [ frame][ square?][ glass?][ OCCUPIED]"

"frame": a jamb is on a LiDAR frame piece, not a wall track (FRAME JAMBS).
"square?": tentative, never yet seen square-on (door_detection, GLASS: SEEN
SQUARE-ON) -- what still keeps it from being confirmed if it was seen often
enough.

Colour is OCCUPIED (orange) or not (green); alpha is the STATE: tentative
faded, confirmed solid, coasting dimmed. A door that dies gets a DELETE on
each of its four slots; there is no DELETEALL, for the reason
/perception/walls/markers dropped it (lidar_front_wall_node, HOW LONG A WALL
MARKER OUTLIVES THE MESSAGE THAT DREW IT): every marker carries a lifetime,
so a stalled node or a restart fades out instead of leaving ghosts, without
the flicker a per-cycle DELETEALL causes.
"""

from f1tenth_perception import door_detection as dd

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker, MarkerArray

NS = 'viz_doors'

SLOT_JAMB0, SLOT_JAMB1, SLOT_LINE, SLOT_TEXT = 0, 1, 2, 3
SLOTS = (SLOT_JAMB0, SLOT_JAMB1, SLOT_LINE, SLOT_TEXT)

DOOR_COLOUR = (0.10, 0.90, 0.40)       # green: an open doorway
OCCUPIED_COLOUR = (1.00, 0.45, 0.00)   # orange: a person is standing in it
ALPHA = {dd.STATE_TENTATIVE: 0.3, dd.STATE_CONFIRMED: 1.0, dd.STATE_COASTING: 0.5}
#: A coasting door is also darker, so dimmed reads as "not seen now" even
#: against a light background, where alpha alone is hard to see.
COASTING_SHADE = 0.6

LINE_WIDTH_M = 0.03
TEXT_SIZE_M = 0.2
TEXT_CLEARANCE_M = 0.2


def set_lifetime(marker, seconds):
    """Set Marker.lifetime from a float, split into sec + nanosec correctly."""
    whole = int(seconds)
    marker.lifetime.sec = whole
    marker.lifetime.nanosec = int(round((seconds - whole) * 1e9))
    return marker


def colour_of(door):
    """Return (r, g, b, a) for a door: occupied or not, by state."""
    r, g, b = OCCUPIED_COLOUR if door.occupied else DOOR_COLOUR
    if door.state == dd.STATE_COASTING:
        r, g, b = r * COASTING_SHADE, g * COASTING_SHADE, b * COASTING_SHADE
    return r, g, b, ALPHA.get(door.state, 0.3)


def label_of(door):
    """Return the text a door is labelled with: id, width, state, leaf, flags."""
    text = f'door {door.id} {door.width:.2f}m {dd.STATE_NAMES.get(door.state, "?")}'
    leaf = getattr(door, 'leaf_side', dd.LEAF_NONE)
    if leaf != dd.LEAF_NONE:
        text += f' leaf {dd.LEAF_NAMES[leaf]}'
    if dd.FRAME_ID in tuple(getattr(door, 'parents', ())):
        text += ' frame'
    if door.state == dd.STATE_TENTATIVE and getattr(door, 'square_looks', 1) == 0:
        text += ' square?'
    if getattr(door, 'glass_suspect', False):
        text += ' glass?'
    return text + (' OCCUPIED' if door.occupied else '')


class DoorMarkerBuilder:
    """Builds /viz/doors arrays and remembers which door ids it drew."""

    def __init__(self, lifetime_s, height_m=2.5, jamb_diameter_m=0.10):
        self.lifetime_s = float(lifetime_s)
        self.height_m = float(height_m)
        self.jamb_diameter_m = float(jamb_diameter_m)
        self.drawn_ids = set()

    def forget(self):
        """Drop the memory of what was drawn (nobody is subscribed any more)."""
        self.drawn_ids = set()

    def build(self, doors, stamp, frame_id):
        """Return one MarkerArray for `doors`, with a DELETE per slot of every dead id."""
        markers = MarkerArray()
        for door in doors:
            markers.markers.extend(self._door_markers(door, stamp, frame_id))
        alive = {door.id for door in doors}
        for dead_id in sorted(self.drawn_ids - alive):
            for slot in SLOTS:
                markers.markers.append(self._delete(dead_id, slot, stamp, frame_id))
        self.drawn_ids = alive
        return markers

    def _header(self, marker, stamp, frame_id, door_id, slot):
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = NS
        marker.id = 4 * int(door_id) + slot
        return marker

    def _delete(self, door_id, slot, stamp, frame_id):
        marker = self._header(Marker(), stamp, frame_id, door_id, slot)
        marker.action = Marker.DELETE
        return marker

    def _door_markers(self, door, stamp, frame_id):
        r, g, b, a = colour_of(door)
        out = []
        for slot, jamb in ((SLOT_JAMB0, door.jamb0), (SLOT_JAMB1, door.jamb1)):
            post = self._header(Marker(), stamp, frame_id, door.id, slot)
            post.type, post.action = Marker.CYLINDER, Marker.ADD
            # A cylinder's pose is its CENTRE: half its height up, so its base
            # stands on the floor.
            post.pose.position = Point(x=float(jamb[0]), y=float(jamb[1]),
                                       z=0.5 * self.height_m)
            post.pose.orientation.w = 1.0
            post.scale.x = post.scale.y = self.jamb_diameter_m
            post.scale.z = self.height_m
            post.color.r, post.color.g, post.color.b, post.color.a = r, g, b, a
            out.append(set_lifetime(post, self.lifetime_s))

        line = self._header(Marker(), stamp, frame_id, door.id, SLOT_LINE)
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = LINE_WIDTH_M
        line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, a
        # Just off the floor, so it is not z-fighting the costmap grid.
        line.points = [Point(x=float(door.jamb0[0]), y=float(door.jamb0[1]), z=0.02),
                       Point(x=float(door.jamb1[0]), y=float(door.jamb1[1]), z=0.02)]
        out.append(set_lifetime(line, self.lifetime_s))

        text = self._header(Marker(), stamp, frame_id, door.id, SLOT_TEXT)
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        centre = door.centre
        text.pose.position = Point(x=float(centre[0]), y=float(centre[1]),
                                   z=self.height_m + TEXT_CLEARANCE_M)
        text.pose.orientation.w = 1.0
        text.scale.z = TEXT_SIZE_M
        text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
        text.text = label_of(door)
        out.append(set_lifetime(text, self.lifetime_s))
        return out
