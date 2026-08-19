import sys
import os
import time
import threading
import urllib.parse
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from core.utils import get_local_ip



class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        exc_type, exc_val, _ = sys.exc_info()
        if exc_type in (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, TimeoutError):
            return
        super().handle_error(request, client_address)


class VideoShareHTTPHandler(BaseHTTPRequestHandler):
    video_path = ""
    video_filename = "Tesla_Dashcam_Clip.mp4"

    def log_message(self, format, *args):
        pass

    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, TimeoutError):
            pass
        except Exception:
            pass

    def do_GET(self):
        try:
            parsed = urllib.parse.urlparse(self.path)
            path = parsed.path
            
            if not os.path.exists(self.video_path):
                self.send_error(404, "Video file not found")
                return

            file_size = os.path.getsize(self.video_path)
            mtime = int(os.path.getmtime(self.video_path))

            if path == "/" or path == "/index.html":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.end_headers()
                
                size_mb = file_size / (1024 * 1024)
                html_content = f"""<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <meta http-equiv="Pragma" content="no-cache">
    <meta http-equiv="Expires" content="0">
    <title>CT 대시캠 모바일 다운로드</title>
    <style>
        * {{ box-sizing: border-box; margin: 0; padding: 0; }}
        body {{
            background-color: #0f1115;
            color: #f0f0f0;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            min-height: 100vh;
            padding: 16px;
        }}
        .header {{
            text-align: center;
            margin-top: 6px;
            margin-bottom: 14px;
        }}
        .header h1 {{
            font-size: 19px;
            font-weight: 700;
            color: #ffffff;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }}
        .header p {{
            font-size: 12px;
            color: #8e95a5;
            margin-top: 4px;
        }}
        .card {{
            background: #1a1d24;
            border: 1px solid #2a2e39;
            border-radius: 16px;
            width: 100%;
            max-width: 480px;
            overflow: hidden;
            box-shadow: 0 8px 24px rgba(0,0,0,0.5);
            padding: 16px;
        }}
        video {{
            width: 100%;
            border-radius: 10px;
            background: #000;
            margin-bottom: 12px;
        }}
        .info-row {{
            display: flex;
            justify-content: space-between;
            font-size: 12px;
            color: #a0a6b5;
            margin-bottom: 14px;
            padding: 0 4px;
        }}
        .info-row strong {{
            color: #00E6FF;
        }}
        .btn-download {{
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
            width: 100%;
            background: linear-gradient(135deg, #0078D7 0%, #00B4D8 100%);
            color: #ffffff;
            text-decoration: none;
            font-size: 16px;
            font-weight: 700;
            padding: 15px;
            border-radius: 12px;
            text-align: center;
            box-shadow: 0 4px 12px rgba(0, 120, 215, 0.4);
            transition: transform 0.1s, opacity 0.2s;
            margin-bottom: 16px;
        }}
        .btn-download:active {{
            transform: scale(0.98);
            opacity: 0.9;
        }}
        .guide-box {{
            background: #14171d;
            border: 1px solid #222630;
            border-radius: 10px;
            padding: 12px;
            font-size: 12px;
            color: #8e95a5;
            line-height: 1.5;
        }}
        .guide-box h3 {{
            font-size: 13px;
            color: #FFA726;
            margin-bottom: 6px;
            display: flex;
            align-items: center;
            gap: 6px;
        }}
        .guide-box ol {{
            margin-left: 18px;
        }}
        .guide-box li {{
            margin-top: 4px;
        }}
    </style>
</head>
<body>
    <div class="header">
        <h1>⚡ CT Dashcam Studio</h1>
        <p>초고속 무선 비디오 전송</p>
    </div>

    <div class="card">
        <video controls playsinline preload="metadata">
            <source src="/video.mp4?t={mtime}" type="video/mp4">
            브라우저가 비디오 재생을 지원하지 않습니다.
        </video>

        <div class="info-row">
            <span>파일명: {self.video_filename}</span>
            <span>용량: <strong>{size_mb:.1f} MB</strong></span>
        </div>

        <a href="/download?t={mtime}" download="{self.video_filename}" class="btn-download">
            💾 비디오 직접 다운로드
        </a>

        <div class="guide-box">
            <h3>💡 스마트폰 갤러리 저장 방법</h3>
            <ol>
                <li>위 <b>[비디오 직접 다운로드]</b> 버튼을 눌러 파일을 다운로드하세요.</li>
                <li>(안드로이드/갤럭시) 다운로드가 완료되면 브라우저 하단의 [열기]를 누르거나 <b>기본 갤러리 앱</b>에서 영상을 감상할 수 있습니다.</li>
                <li>(아이폰/iOS) 사파리 주소창 옆 <b>파란색 다운로드 아이콘(↓)</b>을 누른 뒤, 파일을 열고 좌측 하단 <b>공유 버튼(⎋)</b>을 눌러 <span style="color:#00E6FF">비디오 저장</span>을 선택하시면 기본 사진 앱으로 복사됩니다.</li>
            </ol>
        </div>
    </div>
</body>
</html>"""
                self.wfile.write(html_content.encode("utf-8"))
                return

            elif path == "/download":
                self.send_response(200)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Content-Disposition", f'attachment; filename="{self.video_filename}"')
                self.send_header("Content-Length", str(file_size))
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.end_headers()

                with open(self.video_path, "rb") as f:
                    while chunk := f.read(256 * 1024):
                        self.wfile.write(chunk)
                return

            elif path == "/video.mp4":
                range_header = self.headers.get("Range", None)
                if range_header:
                    try:
                        ranges = range_header.replace("bytes=", "").split("-")
                        start = int(ranges[0]) if ranges[0] else 0
                        end = int(ranges[1]) if len(ranges) > 1 and ranges[1] else file_size - 1
                        end = min(end, file_size - 1)
                        length = end - start + 1

                        self.send_response(206)
                        self.send_header("Content-Type", "video/mp4")
                        self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
                        self.send_header("Content-Length", str(length))
                        self.send_header("Accept-Ranges", "bytes")
                        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0")
                        self.end_headers()

                        with open(self.video_path, "rb") as f:
                            f.seek(start)
                            remaining = length
                            while remaining > 0:
                                chunk_size = min(remaining, 256 * 1024)
                                data = f.read(chunk_size)
                                if not data:
                                    break
                                self.wfile.write(data)
                                remaining -= len(data)
                        return
                    except Exception:
                        pass

                self.send_response(200)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Content-Length", str(file_size))
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0")
                self.end_headers()

                with open(self.video_path, "rb") as f:
                    while chunk := f.read(256 * 1024):
                        self.wfile.write(chunk)
                return

            self.send_error(404, "Not Found")
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, TimeoutError):
            pass
        except Exception:
            pass


class LocalVideoShareServer:
    def __init__(self, video_path, base_port=8765):
        self.video_path = video_path
        self.video_filename = os.path.basename(video_path)
        self.ip = get_local_ip()
        self.port = base_port
        self.server = None
        self.thread = None
        self._start_server()

    def _start_server(self):
        for p in range(self.port, self.port + 50):
            try:
                handler = VideoShareHTTPHandler
                handler.video_path = self.video_path
                handler.video_filename = self.video_filename
                
                self.server = ThreadingHTTPServer(("0.0.0.0", p), handler)
                self.port = p
                break
            except OSError:
                continue
        
        if self.server:
            self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.thread.start()

    @property
    def share_url(self):
        mtime = int(os.path.getmtime(self.video_path)) if os.path.exists(self.video_path) else int(time.time())
        return f"http://{self.ip}:{self.port}/?t={mtime}"

    def stop(self):
        if self.server:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:
                pass
            self.server = None
