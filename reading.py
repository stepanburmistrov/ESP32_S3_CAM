import cv2
import numpy as np
import requests

ESP_IP = "192.168.0.132"            # ваш IP
URL    = f"http://{ESP_IP}/stream"

def stream_jpeg():
    # 3 сек — на само соединение, чтение — без лимита
    resp = requests.get(URL, stream=True, timeout=(3, None))
    resp.raise_for_status()

    buf = b""
    for chunk in resp.iter_content(chunk_size=4096):
        if not chunk:
            continue
        buf += chunk
        # ищем маркеры JPEG SOI/EOI
        while True:
            start = buf.find(b"\xff\xd8")
            end   = buf.find(b"\xff\xd9", start+2)
            if start != -1 and end != -1:
                jpg = buf[start:end+2]
                buf = buf[end+2:]
                yield jpg
            else:
                break

def main():
    cv2.namedWindow("ESP32-CAM", cv2.WINDOW_AUTOSIZE)
    try:
        for jpg in stream_jpeg():
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), 
                                 cv2.IMREAD_COLOR)
            
            if frame is None:
                continue

            frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.imshow("ESP32-CAM", frame)
            if cv2.waitKey(20) == 27:   # Esc
                break
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
