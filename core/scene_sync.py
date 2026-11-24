"""Use common SEI sequence numbers only after validating the counter domain.

Rear streams can have a different counter. Never infer a constant offset from
matching filenames or silently map a missing sequence to a neighbouring frame.
"""
import numpy as np


def presentation_times(path, expected_count):
    """Packet PTS sorted into decoded presentation order; no image decoding.

    MP4 video samples have one frame per packet. Reject ambiguous/missing
    timestamps rather than invent an accurate timing status.
    """
    try:
        import av
        with av.open(str(path)) as container:
            stream=container.streams.video[0]
            values=[float(packet.pts*packet.time_base) for packet in container.demux(stream)
                    if packet.pts is not None]
        times=np.sort(np.asarray(values,float))
        if len(times)!=expected_count or not np.isfinite(times).all() or len(times)<2:
            raise ValueError('PTS/frame count mismatch or insufficient timestamps')
        if times[0]<0 or np.any(np.diff(times)<=0):
            raise ValueError('PTS must be nonnegative and unique')
        return times, dict(status='container_pts',frames=len(times),
            limitation='presentation timestamps; sensor exposure timing not independently measured')
    except Exception as exc:
        # Optional indexing must not prevent the editor from opening a file.
        return None, dict(status='nominal_fps_unverified',reason=f'{type(exc).__name__}: {str(exc)[:200]}')


class SequenceAlignment:
    def __init__(self, reference, target, reference_count, target_count):
        self.mapping = {}
        self.report = dict(status='unverified', reason='incomplete or invalid sequence metadata')
        def valid(cache, count):
            if len(cache) != count or sorted(cache) != list(range(count)):
                return None
            values = [cache[i].get('frame_seq_no') for i in range(count)]
            if any(not isinstance(v, int) or isinstance(v, bool) or v < 0 for v in values):
                return None
            if any(b <= a for a, b in zip(values, values[1:])):
                return None
            return values
        a, b = valid(reference, reference_count), valid(target, target_count)
        if a is None or b is None:
            return
        lookup = {seq: idx for idx, seq in enumerate(b)}
        mapping = {idx: lookup[seq] for idx, seq in enumerate(a) if seq in lookup}
        if len(mapping) < 8 or len(mapping) < .8*min(len(a), len(b)):
            self.report['reason'] = 'counters do not have sufficient common coverage'
            return
        # Matching counters should also describe the same vehicle state.
        checks = []
        for idx in list(mapping)[::max(1,len(mapping)//100)]:
            x, y = reference[idx], target[mapping[idx]]
            fields = ('speed_mps', 'heading', 'steering_deg', 'brake_applied')
            if not all(k in x and k in y for k in fields):
                self.report['reason'] = 'vehicle state is missing'
                return
            # SEI vehicle-state refresh and exposure are not an atomic sample.
            # Check compatibility, and report that this is not a timing survey.
            delta = {k: abs(float(x[k])-float(y[k])) for k in fields}
            delta['heading'] = abs((float(x['heading'])-float(y['heading'])+180)%360-180)
            checks.append(all(np.isfinite(float(x[k])) and np.isfinite(float(y[k])) for k in fields)
                          and delta['speed_mps'] <= .1 and delta['heading'] <= 2.
                          and delta['steering_deg'] <= 2. and delta['brake_applied'] == 0.)
        agreement = sum(checks)/len(checks)
        if agreement < .95:
            self.report.update(reason='matching counters disagree on vehicle state', agreement=agreement)
            return
        self.mapping = mapping
        self.report = dict(status='sequence_matched', matched_frames=len(mapping),
                           reference_frames=len(a), target_frames=len(b),
                           vehicle_state_agreement=agreement,
                           limitation='common counter verified; exposure timing not independently measured')

    def target_index(self, reference_index):
        return self.mapping.get(reference_index)
