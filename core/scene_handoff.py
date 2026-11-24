"""Experimental bidirectional front/side handoffs with two-sided motion evidence."""
from collections import defaultdict
from dataclasses import replace
import numpy as np


def stitch_handoffs(scenes, rows, poses, audits, camera_widths):
    tracks = defaultdict(list)
    for i, scene in enumerate(scenes):
        for actor in scene.actors:
            if actor.tracking_state == 'observed':
                tracks[actor.key].append((i, actor))

    def point(i, actor):
        return poses[i][0] + poses[i][1] @ np.array([actor.x, actor.z])

    def boundary(i, actor, camera, edge):
        data = rows[i]['cameras'].get(camera, {})
        if abs(data.get('source_time_s', scenes[i].time)-scenes[i].time) > .08:
            return False
        for obj in data.get('objects', []):
            if obj['track_id'] not in actor.sources:
                continue
            x, y, w, h = obj['bbox']
            width = camera_widths.get(camera)
            if width is None:
                return False
            margin = width * .003125
            if (edge == 'left' and x < margin) or (edge == 'right' and x+w > width-margin):
                return True
        return False

    def velocity(history, endpoint, before):
        samples = [(k, a) for k, a in history
                   if 0 <= (scenes[endpoint].time-scenes[k].time)*(1 if before else -1) <= .8]
        if len(samples) < 5 or scenes[samples[-1][0]].time-scenes[samples[0][0]].time < .4:
            return None
        times = np.array([scenes[k].time-scenes[endpoint].time for k, _ in samples])
        points = np.array([point(k, a) for k, a in samples])
        design = np.column_stack([np.ones(len(times)), times])
        fit = np.linalg.lstsq(design, points, rcond=None)[0]
        if np.max(np.linalg.norm(design@fit-points, axis=1)) > 1.:
            return None
        return fit[1]

    proposals = []
    for old, history in tracks.items():
        if len(history) < 5:
            continue
        i, a = history[-1]
        side = next((c for c in ('left_repeater', 'right_repeater')
                     if any(s.startswith(c+':') for s in a.sources)), None)
        for new, future in tracks.items():
            if new == old or len(future) < 5:
                continue
            j, b = future[0]
            gap = scenes[j].time-scenes[i].time
            if not .1 < gap <= 4 or a.kind != b.kind or a.x*b.x <= 0:
                continue
            incoming_side = next((c for c in ('left_repeater', 'right_repeater')
                                  if any(s.startswith(c+':') for s in b.sources)), None)
            route_side = side or incoming_side
            if route_side is None:
                continue
            side_edge = 'right' if route_side == 'left_repeater' else 'left'
            front_edge = 'left' if route_side == 'left_repeater' else 'right'
            exit_camera, exit_edge = (side, side_edge) if side else ('front', front_edge)
            entry_camera, entry_edge = ('front', front_edge) if side else (incoming_side, side_edge)
            if not boundary(i, a, exit_camera, exit_edge) or not boundary(j, b, entry_camera, entry_edge):
                continue
            if any(scenes[k+1].time-scenes[k].time > .3 or
                   any('tracks reset' in e.get('reason', '') for e in audits[k].get('world_track_events', []))
                   for k in range(i, j)):
                continue
            # Longer blind intervals require both past and future motion support.
            # This does not increase the online ghost lifetime.
            predicted = next((x for x in scenes[j].actors if x.key == old and x.tracking_state == 'predicted'), None)
            if gap > 2 or predicted is None:
                v0, v1 = velocity(history, i, True), velocity(future, j, False)
                if v0 is None or v1 is None or np.linalg.norm(v0-v1) > 6:
                    continue
                displacement = point(j, b)-point(i, a)
                # Curved roads change world velocity direction. Compare the
                # displacement with both endpoint velocities (trapezoidal
                # integration), rather than demanding straight extrapolation
                # from either endpoint across the entire interval.
                if np.linalg.norm(v1-v0)/gap > 3 or np.linalg.norm(displacement-(v0+v1)*gap/2) > 5:
                    continue
            elif np.linalg.norm(point(j, predicted)-point(j, b)) > 5:
                continue
            if np.linalg.norm(point(j, b)-point(i, a))/gap > 55:
                continue
            proposals.append((old, new, i, j, a, b))
    # Mutual uniqueness: ambiguous traffic remains unresolved.
    counts = defaultdict(int)
    for old, new, *_ in proposals:
        counts[old] += 1
        counts[new] += 1
    accepted = 0
    for old, new, i, j, a, b in proposals:
        if counts[old] != 1 or counts[new] != 1:
            continue
        points = {}
        for k in range(i+1, j):
            q = (scenes[k].time-scenes[i].time)/(scenes[j].time-scenes[i].time)
            local = poses[k][1].T @ ((1-q)*point(i, a)+q*point(j, b)-poses[k][0])
            if any(x.key not in (old, new) and abs(local[0]-x.x)<(a.width+x.width)/2
                   and abs(local[1]-x.z)<(a.length+x.length)/2 for x in scenes[k].actors):
                break
            points[k] = local
        else:
            for k in range(i+1, len(scenes)):
                scene = scenes[k]
                scene.actors = [replace(x, key=old) if x.key == new else x
                                for x in scene.actors if not (x.key == old and k >= j)]
                if k in points:
                    scene.actors = [x for x in scene.actors if x.key != old]
                    x, z = points[k]
                    scene.actors.append(replace(a, x=float(x), z=float(z), sources=(),
                        tracking_state='interpolated', prediction_age=scene.time-scenes[i].time,
                        prediction_horizon=scenes[j].time-scenes[i].time,
                        position_uncertainty=max(a.position_uncertainty,b.position_uncertainty)+1.))
                audits[k].setdefault('handoffs', []).append({'from':old,'to':new,
                    'method':'unique front/side boundary handoff with temporal motion evidence',
                    'past_time_s':scenes[i].time,'future_time_s':scenes[j].time})
            accepted += 1
    if accepted:
        for scene, audit in zip(scenes, audits):
            audit['visible_ids'] = [a.key for a in scene.actors]
            audit['world_tracks'] = [{'id':a.key,'position':[a.x,a.z],
                'state':a.tracking_state,'prediction_age_s':a.prediction_age,
                'conditional_sigma_m':a.position_uncertainty,'sources':a.sources}
                for a in scene.actors]
    return accepted
