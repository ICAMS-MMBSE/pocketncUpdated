from flask import Flask, Response
import cv2
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Must be the device's actual V4L2 capture node (check with `v4l2-ctl --list-devices`),
# passed as a string path — an int index makes OpenCV enumerate/guess and can fail
# with "can't open camera by index" even when the device is fine.
camera = cv2.VideoCapture("/dev/video19", cv2.CAP_V4L2)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            logger.warning("Failed to read frame")
            break
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/pocketNc/stream')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # debug=True spawns the module twice via Flask's reloader, which opens the
    # camera twice and causes conflicts — keep this False.
    app.run(debug=False, port=8000, host="0.0.0.0")
