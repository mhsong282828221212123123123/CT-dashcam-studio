import sys
import os
import re
import socket


def parse_version_tuple(v_str):
    """ 'v1.1.0', '1.1', 'V2.0.1' 등의 문자열을 (1, 1, 0) 형태의 정수 튜플로 변환 """
    if not v_str:
        return (0, 0, 0)
    nums = [int(x) for x in re.findall(r'\d+', str(v_str))]
    while len(nums) < 3:
        nums.append(0)
    return tuple(nums)


def is_newer_version(latest_tag, current_ver):
    """ 깃허브 릴리즈 태그 버전이 현재 앱 버전보다 높은 경우(초과)에만 True 반환 """
    return parse_version_tuple(latest_tag) > parse_version_tuple(current_ver)


def resource_path(relative_path):
    """ PyInstaller 번들 및 일반 실행 환경에서 안전하게 리소스 절대경로 반환 """
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(base_path, relative_path)


def get_app_dir():
    """ Nuitka onefile, PyInstaller, 일반 파이썬 환경에서 정확한 앱 실행 폴더 반환 """
    # 1. Nuitka onefile 환경 변수
    nuitka_bin = os.environ.get("NUITKA_ONEFILE_BINARY")
    if nuitka_bin and os.path.exists(nuitka_bin):
        return os.path.dirname(os.path.abspath(nuitka_bin))
    
    # 2. PyInstaller 또는 일반 exe 빌드 환경 (Frozen)
    if getattr(sys, 'frozen', False):
        exe_cand = os.path.abspath(sys.executable)
        exe_lower = exe_cand.lower()
        if not ("appdata" in exe_lower and "temp" in exe_lower and ("onefile_" in exe_lower or "_mei" in exe_lower)):
            return os.path.dirname(exe_cand)
        if sys.argv and sys.argv[0] and os.path.exists(sys.argv[0]):
            return os.path.dirname(os.path.abspath(sys.argv[0]))

    # 3. 일반 파이썬 스크립트(.py) 실행 환경
    # 메인 스크립트 sys.argv[0] 위치 확인
    if sys.argv and sys.argv[0] and sys.argv[0] not in ('-c', '-m', ''):
        cand = os.path.abspath(sys.argv[0])
        if os.path.exists(cand):
            return cand if os.path.isdir(cand) else os.path.dirname(cand)
            
    # core/utils.py 상위 프로젝트 루트 폴더 기준
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if os.path.exists(project_root):
        return project_root

    return os.getcwd()


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"
