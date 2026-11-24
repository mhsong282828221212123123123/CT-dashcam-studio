"""Separate numeric odometry input from rounded dashboard labels."""
import math


def odometry_speed_kmh(data):
    if not data:
        return None
    if 'speed_mps' in data:
        value=data['speed_mps']
        factor=3.6
    else:
        value=data.get('speed_kmh')
        factor=1.
    if not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value):
        return None
    speed=float(value)*factor
    return speed if 0<=speed<250 else None
