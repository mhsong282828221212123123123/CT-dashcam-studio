"""Reproduce documented CURRENT behavior; this task does not fix QR networking.

Only synthetic files and loopback test sockets are used. Production bind/start/
stop methods are untouched. No real footage or external hosts are contacted.
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.request import urlopen, Request

from PyQt6.QtWidgets import QApplication
from network.server import ThreadingHTTPServer, VideoShareHTTPHandler
from ui.dialogs import QRShareDialog


class QRFindingsTests(unittest.TestCase):
    def test_current_handler_serves_synthetic_file_without_credentials(self):
        with tempfile.TemporaryDirectory() as temporary:
            file = Path(temporary)/'synthetic.mp4'; file.write_bytes(b'synthetic-test-video-only')
            with patch.object(VideoShareHTTPHandler, 'video_path', str(file)):
                server = ThreadingHTTPServer(('127.0.0.1', 0), VideoShareHTTPHandler)
                thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
                try:
                    base = f'http://127.0.0.1:{server.server_address[1]}'
                    with urlopen(base+'/download', timeout=3) as response:
                        self.assertEqual(response.status, 200)
                        self.assertEqual(response.read(), file.read_bytes())
                    with urlopen(Request(base+'/video.mp4', headers={'Range':'bytes=0-3'}),timeout=3) as response:
                        self.assertEqual(response.status, 206)
                        self.assertEqual(response.read(), b'synt')
                finally:
                    server.shutdown(); server.server_close(); thread.join(timeout=3)

    def test_current_dialog_accept_reject_do_not_invoke_stop_but_close_does(self):
        app = QApplication.instance() or QApplication([])
        class TestServer:
            def __init__(self, path):
                self.ip = '127.0.0.1'; self.share_url = 'http://127.0.0.1:12345/'; self.stops = 0
            def stop(self):
                self.stops += 1
        with tempfile.TemporaryDirectory() as temporary, patch('ui.dialogs.LocalVideoShareServer', TestServer):
            file = Path(temporary)/'synthetic.mp4'; file.write_bytes(b'test')
            for action, expected in [('accept',0),('reject',0),('close',1)]:
                dialog = QRShareDialog(str(file)); dialog.show(); app.processEvents()
                getattr(dialog, action)(); app.processEvents()
                self.assertEqual(dialog.server.stops, expected, action)
                dialog.deleteLater(); app.processEvents()

    def test_current_handler_file_state_is_shared_across_servers(self):
        with tempfile.TemporaryDirectory() as temporary:
            a=Path(temporary)/'a.mp4';a.write_bytes(b'A synthetic')
            b=Path(temporary)/'b.mp4';b.write_bytes(b'B synthetic')
            with patch.object(VideoShareHTTPHandler, 'video_path', str(a)):
                servers=[ThreadingHTTPServer(('127.0.0.1',0),VideoShareHTTPHandler) for _ in range(2)]
                threads=[threading.Thread(target=s.serve_forever,daemon=True) for s in servers]
                for thread in threads:thread.start()
                try:
                    old=f'http://127.0.0.1:{servers[0].server_address[1]}/download'
                    with urlopen(old,timeout=3) as response:self.assertEqual(response.read(),a.read_bytes())
                    VideoShareHTTPHandler.video_path=str(b)
                    with urlopen(old,timeout=3) as response:self.assertEqual(response.read(),b.read_bytes())
                finally:
                    for server in servers:server.shutdown();server.server_close()
                    for thread in threads:thread.join(timeout=3)


if __name__ == '__main__':
    unittest.main()
