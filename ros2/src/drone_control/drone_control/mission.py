#!/usr/bin/env python3
"""
Mission parsing for MAVROS.
Parses QGC WPL 110 format into MAVROS Waypoint messages.
"""

from mavros_msgs.msg import Waypoint


def parse_qgc_wpl(wpl_text: str) -> list[Waypoint]:
    """
    Parse QGC WPL 110 format into MAVROS Waypoint list.

    Format: index current frame command p1 p2 p3 p4 lat lon alt autocontinue
    """
    waypoints = []
    lines = wpl_text.strip().split('\n')

    if not lines or not lines[0].startswith('QGC WPL'):
        raise ValueError("Invalid QGC WPL format")

    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue

        parts = line.split('\t')
        if len(parts) < 12:
            parts = line.split()
        if len(parts) < 12:
            continue

        wp = Waypoint()
        wp.frame = int(parts[2])
        wp.command = int(parts[3])
        wp.is_current = int(parts[1]) == 1
        wp.autocontinue = int(parts[11]) == 1
        wp.param1 = float(parts[4])
        wp.param2 = float(parts[5])
        wp.param3 = float(parts[6])
        wp.param4 = float(parts[7])
        wp.x_lat = float(parts[8])
        wp.y_long = float(parts[9])
        wp.z_alt = float(parts[10])

        waypoints.append(wp)

    return waypoints
