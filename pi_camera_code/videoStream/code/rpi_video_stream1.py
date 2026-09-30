import cv2
import requests
import time
import argparse
import subprocess
from threading import Thread
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class VideoStreamer:
    def __init__(self, stream_url, stream_type='http', camera_index=0, width=640, height=480, fps=30, quality=80, rtmp_direct=False):
        """
        Initialize the video streamer
        
        Args:
            stream_url: URL to stream video to
            stream_type: Type of streaming ('http' or 'rtmp')
            camera_index: Camera device index (default 0)
            width: Frame width
            height: Frame height
            fps: Frames per second
            quality: JPEG compression quality (0-100, for HTTP only)
            rtmp_direct: If True, use FFmpeg direct camera access. If False, pipe frames from OpenCV
        """
        self.stream_url = stream_url
        self.stream_type = stream_type.lower()
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.rtmp_direct = rtmp_direct
        self.running = False
        self.cap = None
        self.ffmpeg_process = None
        
    def start(self):
        """Start the video streaming"""
        self.running = True
        
        # For RTMP with direct mode, FFmpeg accesses camera directly - no need for OpenCV
        if self.stream_type == 'rtmp' and self.rtmp_direct:
            logger.info(f"Using camera /dev/video{self.camera_index} (direct mode)")
            logger.info(f"Streaming to: {self.stream_url} (type: {self.stream_type})")
            return self.start_rtmp_direct()
        
        # For HTTP or RTMP piped mode, we need OpenCV
        logger.info(f"Initializing camera {self.camera_index}...")
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            logger.error("Failed to open camera")
            return False
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        logger.info(f"Camera initialized: {self.width}x{self.height} @ {self.fps}fps")
        logger.info(f"Streaming to: {self.stream_url} (type: {self.stream_type})")
        
        if self.stream_type == 'rtmp':
            return self.start_rtmp_piped()
        else:
            return self.start_http_stream()
    
    def start_rtmp_direct(self):
        """Start RTMP streaming using FFmpeg directly with v4l2"""
        # FFmpeg command for RTMP streaming - uses v4l2 directly (no piping)
        command = [
            'ffmpeg',
            '-f', 'lavfi',
            '-i', 'anullsrc',
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-framerate', str(self.fps),
            '-video_size', f'{self.width}x{self.height}',
            '-i', f'/dev/video{self.camera_index}',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-c:a', 'aac',
            '-f', 'flv',
            self.stream_url
        ]
        
        try:
            logger.info("Starting FFmpeg process for RTMP streaming (direct mode)...")
            logger.info(f"Command: {' '.join(command)}")
            
            self.ffmpeg_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            logger.info("FFmpeg process started. Streaming...")
            logger.info("Press Ctrl+C to stop")
            
            # Monitor the process
            while self.running:
                # Check if process is still running
                if self.ffmpeg_process.poll() is not None:
                    # Process has ended
                    _, stderr = self.ffmpeg_process.communicate()
                    logger.error(f"FFmpeg process ended unexpectedly")
                    logger.error(f"FFmpeg stderr: {stderr.decode()}")
                    break
                
                time.sleep(1)
                    
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except FileNotFoundError:
            logger.error("FFmpeg not found. Please install FFmpeg: sudo apt-get install ffmpeg")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.stop()
            
        return True
    
    def start_rtmp_piped(self):
        """Start RTMP streaming by piping frames from OpenCV to FFmpeg"""
        # FFmpeg command for RTMP streaming with piped input
        # Using same encoding options as your working command
        command = [
            'ffmpeg',
            '-f', 'lavfi',
            '-i', 'anullsrc',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-c:a', 'aac',
            '-f', 'flv',
            self.stream_url
        ]
        
        try:
            logger.info("Starting FFmpeg process for RTMP streaming (piped mode)...")
            logger.info(f"Command: {' '.join(command)}")
            
            self.ffmpeg_process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            
            frame_count = 0
            error_count = 0
            max_errors = 10
            
            logger.info("FFmpeg process started. Streaming frames...")
            
            while self.running:
                ret, frame = self.cap.read()
                
                if not ret:
                    logger.warning("Failed to capture frame")
                    error_count += 1
                    if error_count >= max_errors:
                        logger.error("Too many capture errors, stopping...")
                        break
                    time.sleep(0.1)
                    continue
                
                error_count = 0
                
                try:
                    # Write frame to FFmpeg stdin
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                    frame_count += 1
                    
                    if frame_count % 100 == 0:
                        logger.info(f"Streamed {frame_count} frames via RTMP")
                        
                except BrokenPipeError:
                    logger.error("FFmpeg process closed unexpectedly (BrokenPipeError)")
                    # Try to get error output
                    try:
                        _, stderr = self.ffmpeg_process.communicate(timeout=2)
                        logger.error(f"FFmpeg stderr: {stderr.decode()}")
                    except:
                        pass
                    break
                except Exception as e:
                    logger.error(f"Error writing frame to FFmpeg: {e}")
                    break
                    
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except FileNotFoundError:
            logger.error("FFmpeg not found. Please install FFmpeg: sudo apt-get install ffmpeg")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.stop()
            
        return True
    
    def start_http_stream(self):
        """Start HTTP POST streaming"""
        frame_count = 0
        error_count = 0
        max_errors = 10
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                
                if not ret:
                    logger.warning("Failed to capture frame")
                    error_count += 1
                    if error_count >= max_errors:
                        logger.error("Too many capture errors, stopping...")
                        break
                    time.sleep(0.1)
                    continue
                
                error_count = 0  # Reset error count on successful capture
                
                # Encode frame as JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
                _, buffer = cv2.imencode('.jpg', frame, encode_param)
                
                # Send frame to URL
                try:
                    response = requests.post(
                        self.stream_url,
                        files={'frame': ('frame.jpg', buffer.tobytes(), 'image/jpeg')},
                        timeout=5
                    )
                    
                    if response.status_code == 200:
                        frame_count += 1
                        if frame_count % 100 == 0:
                            logger.info(f"Streamed {frame_count} frames")
                    else:
                        logger.warning(f"Server returned status code: {response.status_code}")
                        
                except requests.exceptions.RequestException as e:
                    logger.error(f"Failed to send frame: {e}")
                    time.sleep(1)  # Wait before retrying
                
                # Control frame rate
                time.sleep(1.0 / self.fps)
                
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.stop()
            
        return True
    
    def stop(self):
        """Stop the video streaming and release resources"""
        logger.info("Stopping video stream...")
        self.running = False
        
        if self.cap is not None:
            self.cap.release()
        
        if self.ffmpeg_process is not None:
            try:
                logger.info("Terminating FFmpeg process...")
                if self.ffmpeg_process.stdin:
                    self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("FFmpeg didn't terminate gracefully, killing process...")
                self.ffmpeg_process.kill()
            except Exception as e:
                logger.warning(f"Error closing FFmpeg process: {e}")
        
        logger.info("Video stream stopped")

def main():
    parser = argparse.ArgumentParser(description='Stream video from Raspberry Pi camera')
    parser.add_argument('--url', help='URL to stream video to')
    parser.add_argument('--type', choices=['http', 'rtmp'], default='http', 
                        help='Streaming type: http or rtmp (default: http)')
    parser.add_argument('--rtmp-direct', action='store_true',
                        help='For RTMP: use direct camera access (default is to pipe frames)')
    parser.add_argument('--camera', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=640, help='Frame width (default: 640)')
    parser.add_argument('--height', type=int, default=480, help='Frame height (default: 480)')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second (default: 30)')
    parser.add_argument('--quality', type=int, default=80, help='JPEG quality 0-100 for HTTP (default: 80)')
    
    args = parser.parse_args()
    
    # Create and start streamer
    streamer = VideoStreamer(
        stream_url=args.url,
        stream_type=args.type,
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
        rtmp_direct=args.rtmp_direct
    )
    
    streamer.start()

if __name__ == '__main__':
    main()
