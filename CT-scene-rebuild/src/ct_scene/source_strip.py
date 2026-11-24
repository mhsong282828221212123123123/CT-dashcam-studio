"""Known source recording coverage; never present a held tail as live imagery."""
import math


def status(coverage,camera,time_s):
    row=coverage.get(camera,{})
    start=row.get('first_pts_s');end=row.get('end_pts_s')
    if start is None or end is None or not all(math.isfinite(x) for x in [start,end]) or end<=start:return 'unknown'
    if time_s<start:return 'recording_not_started'
    if time_s>end:return 'recording_ended'
    return 'available'
