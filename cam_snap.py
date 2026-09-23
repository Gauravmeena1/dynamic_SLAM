#!/usr/bin/env python3
"""在 gateway_slam container 裡跑：從建圖相機（D435；序號由 KACHAKA_CAMERA_SERIAL 設定）抓一幀存 PNG，
或開一個 MJPEG 串流讓瀏覽器即時看。由 arm_cam_tune.sh 用 `docker exec -i` 餵進去執行。

    python3 -u - snap /tmp/out.png              # 抓一幀（跳過前 12 幀等自動曝光穩定）
    python3 -u - live 8091                      # MJPEG 串流，瀏覽器開 http://<Thor IP>:8091/

⚠️ 跟 SLAM 的 collector 搶同一台相機，只能在兩趟之間用；建圖進行中要看畫面請用
   fungi 的 live_view.py（它只輪詢 run 目錄的 rgb/，不碰相機）。
"""
import argparse
import os

SER = os.getenv("KACHAKA_CAMERA_SERIAL")
W, H, FPS = 640, 480, 15


def camera_modules():
    """選用相依套件延後載入，讓 --help 在一般電腦也能使用。

    Load optional camera dependencies lazily so --help works on non-camera hosts.
    """
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ModuleNotFoundError as exc:
        raise SystemExit(
            f"缺少選用相依套件 / Missing optional dependency: {exc.name}. "
            "Install requirements-optional.txt on the camera host."
        ) from exc
    return cv2, np, rs


def open_pipeline():
    _, _, rs = camera_modules()
    if not SER:
        raise SystemExit("請先在 .env 設定 KACHAKA_CAMERA_SERIAL")
    p = rs.pipeline()
    c = rs.config()
    c.enable_device(SER)
    c.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    p.start(c)
    return p


def snap(out):
    cv2, np, _ = camera_modules()
    p = open_pipeline()
    try:
        for _ in range(12):                       # 自動曝光要幾幀才穩
            f = p.wait_for_frames(5000)
        img = np.asanyarray(f.get_color_frame().get_data())
        # 畫一條 10% 高度的參考線：建圖時被支架佔掉的大約就是這一帶
        cv2.line(img, (0, int(H * 0.9)), (W, int(H * 0.9)), (0, 0, 255), 1)
        cv2.imwrite(out, img)
        print(f"saved {out} {img.shape}")
    finally:
        p.stop()


def live(port):
    cv2, np, _ = camera_modules()
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    p = open_pipeline()
    state = {"jpg": None}

    # 類別名不能叫 H：會在方法裡遮掉模組層的高度常數 H，int(H*0.9) 直接 TypeError（踩過）
    class StreamHandler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path != "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"<html><body style='margin:0;background:#000'>"
                                 b"<img src='/stream' style='width:100vw'></body></html>")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    f = p.wait_for_frames(5000)
                    img = np.asanyarray(f.get_color_frame().get_data())
                    cv2.line(img, (0, int(H * 0.9)), (W, int(H * 0.9)), (0, 0, 255), 1)
                    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg.tobytes() + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer(("0.0.0.0", port), StreamHandler)
    print(f"live on :{port}  (Ctrl+C / arm_cam_tune.sh live-stop 結束)", flush=True)
    try:
        srv.serve_forever()
    finally:
        p.stop()


def main():
    parser = argparse.ArgumentParser(
        description="RealSense 單幀／串流工具 / RealSense snapshot and MJPEG utility")
    sub = parser.add_subparsers(dest="command")
    snap_parser = sub.add_parser("snap", help="抓一幀 / Save one frame")
    snap_parser.add_argument("output", nargs="?", default="/tmp/cam_snap.png",
                             help="輸出 PNG / Output PNG path")
    live_parser = sub.add_parser("live", help="啟動 MJPEG 串流 / Start an MJPEG stream")
    live_parser.add_argument("port", nargs="?", type=int, default=8091,
                             help="HTTP port (default: 8091)")
    args = parser.parse_args()
    command = args.command or "snap"
    if command == "snap":
        snap(getattr(args, "output", "/tmp/cam_snap.png"))
    else:
        live(args.port)


if __name__ == "__main__":
    main()
