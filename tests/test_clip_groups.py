import unittest
from datetime import datetime, timedelta
from core.clip_groups import group_recent_clips, clip_datetime, clip_identity, group_title


def clips(count, gap=60, start=datetime(2026,10,30,12), folder="/a"):
    return [dict(prefix=(start+timedelta(seconds=i*gap)).strftime("%Y-%m-%d_%H-%M-%S"), dirpath=folder, cams={"front":f"{folder}/{i}.mp4"}) for i in range(count)]


class ClipGroupingTests(unittest.TestCase):
    def test_count_boundaries(self):
        for count, expected in {0:[],1:[1],10:[10],11:[11],12:[12],13:[10,3],20:[10,10],21:[10,11],22:[10,12],23:[10,10,3]}.items():
            with self.subTest(count=count):
                source=clips(count)
                grouped=group_recent_clips(source)
                self.assertEqual([len(group) for group in grouped],expected)
                self.assertEqual([id(c) for group in grouped for c in group],[id(c) for c in source])

    def test_start_time_gap_boundaries(self):
        for gap,expected in ((179,[2]),(180,[1,1]),(181,[1,1])):
            with self.subTest(gap=gap):
                self.assertEqual([len(g) for g in group_recent_clips(clips(2,gap))],expected)

    def test_sorted_without_mutating_input(self):
        source=list(reversed(clips(13)));before=source[:]
        grouped=group_recent_clips(source)
        self.assertEqual(source,before)
        self.assertEqual([clip_datetime(c) for g in grouped for c in g],sorted(clip_datetime(c) for c in source))

    def test_session_tails_are_never_combined(self):
        source=clips(11)+clips(2,start=datetime(2026,10,30,14))
        self.assertEqual([len(g) for g in group_recent_clips(source)],[11,2])

    def test_midnight_is_continuous_and_title_includes_both_dates(self):
        source=clips(3,start=datetime(2026,10,30,23,59))
        self.assertEqual(len(group_recent_clips(source)),1)
        self.assertEqual(group_title(source),'2026-10-30 23:59:00 – 2026-10-31 00:01:00')

    def test_title_does_not_assume_a_minute_duration(self):
        self.assertEqual(group_title(clips(1)),'2026-10-30 12:00:00 – 12:00:00')

    def test_path_identity_survives_same_timestamp(self):
        a,b=clips(1,folder='/a')[0],clips(1,folder='/b')[0]
        self.assertNotEqual(clip_identity(a),clip_identity(b))
        self.assertEqual(len(group_recent_clips([a,b])[0]),2)

    def test_invalid_dates_are_isolated(self):
        source=[{'prefix':'bad'},{'prefix':'2026-99-99_12-00-00'}]+clips(1)
        self.assertEqual([len(g) for g in group_recent_clips(source)],[1,1,1])


if __name__=='__main__': unittest.main()
