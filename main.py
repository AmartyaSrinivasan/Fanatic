import os

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
if os.path.exists(EDGE):
    os.environ.setdefault("BROWSER", f'"{EDGE}" --app=%s')

from fanatic.app import run

if __name__ == "__main__":
    run()