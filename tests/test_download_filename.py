from pathlib import Path
import tempfile
import threading
import unittest
from urllib.parse import unquote
from urllib.request import urlopen

from network.server import ThreadingHTTPServer, VideoShareHTTPHandler


class DownloadFilenameTests(unittest.TestCase):
    def test_unicode_and_special_filenames_download_with_original_name(self):
        for filename in ('한글 영상.mp4', '영상 🚘.mp4', 'clip & 100%.mp4'):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as directory:
                video = Path(directory) / filename
                video.write_bytes(b'synthetic video')
                class Handler(VideoShareHTTPHandler):
                    video_path = str(video)
                    video_filename = filename
                server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                try:
                    with urlopen(f'http://127.0.0.1:{server.server_port}/download', timeout=3) as response:
                        self.assertEqual(response.status, 200)
                        self.assertEqual(response.read(), video.read_bytes())
                        disposition = response.headers['Content-Disposition']
                        disposition.encode('ascii')
                        self.assertIn('filename="Tesla_Dashcam_Clip.mp4"', disposition)
                        self.assertEqual(unquote(disposition.split("filename*=UTF-8''", 1)[1]), filename)
                finally:
                    server.shutdown()
                    server.server_close()
                    thread.join(timeout=3)
