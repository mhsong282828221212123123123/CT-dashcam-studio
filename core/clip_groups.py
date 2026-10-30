"""Presentation-only grouping; clip dictionaries and source paths are preserved."""
from datetime import datetime
import re


def clip_datetime(clip):
    match = re.match(r"\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}", clip.get("prefix", ""))
    try:
        return datetime.strptime(match.group(), "%Y-%m-%d_%H-%M-%S") if match else datetime.min
    except ValueError:
        return datetime.min


def clip_identity(clip):
    return (clip.get("dirpath", ""), clip.get("prefix", ""), tuple(sorted(clip.get("cams", {}).items())))


def group_recent_clips(clips):
    ordered = sorted(clips, key=lambda c: (clip_datetime(c), clip_identity(c)))
    sessions = []
    for clip in ordered:
        dt = clip_datetime(clip)
        if not sessions or dt == datetime.min or clip_datetime(sessions[-1][-1]) == datetime.min or (dt - clip_datetime(sessions[-1][-1])).total_seconds() >= 180:
            sessions.append([])
        sessions[-1].append(clip)
    groups = []
    for session in sessions:
        chunks = [session[i:i+10] for i in range(0, len(session), 10)]
        if len(chunks) > 1 and len(chunks[-1]) <= 2:
            chunks[-2].extend(chunks.pop())
        groups.extend(chunks)
    return groups


def group_title(clips):
    """Range of source START timestamps, without inventing a final duration."""
    first, last = clip_datetime(clips[0]), clip_datetime(clips[-1])
    if first == datetime.min or last == datetime.min:
        return clips[0].get("prefix", "")
    end_format = "%H:%M:%S" if first.date() == last.date() else "%Y-%m-%d %H:%M:%S"
    return first.strftime("%Y-%m-%d %H:%M:%S") + " – " + last.strftime(end_format)
