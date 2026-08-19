import json
import urllib.request
from PyQt6.QtCore import QThread, pyqtSignal
from core.constants import APP_VERSION, GITHUB_REPO
from core.utils import is_newer_version


class UpdateCheckWorker(QThread):
    update_available = pyqtSignal(str, str)
    
    def run(self):
        try:
            url = f"https://api.github.com/repos/{GITHUB_REPO}/releases/latest"
            req = urllib.request.Request(url, headers={'User-Agent': 'CT-Dashcam-Studio'})
            with urllib.request.urlopen(req, timeout=5) as response:
                data = json.loads(response.read().decode())
                
            tag_name = data.get("tag_name", "")
            if is_newer_version(tag_name, APP_VERSION):
                html_url = data.get("html_url", "")
                if html_url:
                    self.update_available.emit(tag_name, html_url)
        except Exception as e:
            print(f"Update check failed: {e}")
