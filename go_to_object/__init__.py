"""go_to_object -- drive a car-like vehicle to a detected point target.

Frame conventions, fixed across every module in this package:

  * The world frame is ENU-like and right-handed: x east, y north, z up.
  * Headings (``psi``) are CCW from +x, in radians, wrapped to ``(-pi, pi]``
    by the single :func:`~go_to_object.pursuit_geometry.wrap_pi` helper.
  * Positive curvature turns left (CCW); a positive bearing means the object
    is to the left of the vehicle.
  * Distances are metres, times are ``float`` seconds, angles are radians.

Everything crossing a module boundary is expressed in the world frame.
Camera- and body-frame quantities are converted exactly once, at ingest
(:meth:`~go_to_object.object_tracker.ObjectTracker.push_detection`).

``pursuit_geometry`` and ``object_tracker`` import without a ROS environment;
only ``mission_node`` needs rclpy.
"""
