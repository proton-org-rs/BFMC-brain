# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

"""
Lane Detection Module for Autonomous Driving

This module provides lane detection functionality using frames from the central
camera process (processCamera/threadCamera). It subscribes to the laneCamera
message queue to receive frames, eliminating the need for a separate camera
initialization.

Features:
- Receives frames from central camera via message queue
- Performs perspective transform to bird's eye view
- Detects lane lines using Hough transform
- Calculates steering angle using PD controller

Usage:
    lane_detector = LaneDetection(queueList, logger)
    lane_detector.start()
    
    # In main loop:
    steering_angle = lane_detector.get_steering_angle()
    
    # On shutdown:
    lane_detector.stop()
"""

import sys
import os
import time
import numpy as np
import cv2
import picamera2
from ultralytics import YOLO
from collections import deque, Counter


# Add parent directory to path for direct execution
if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from src.utils.messages.allMessages import laneCamera
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber


# ===================================== LANE CONTROLLER =====================================
class LaneController:
    """PD + theta controller for lane keeping."""
    
    def __init__(self, Kp, Kd, Kt):
        """
        Initialize the lane controller.
        
        Args:
            Kp: Proportional gain for lateral error
            Kd: Derivative gain for lateral error change
            Kt: Gain for heading angle error
        """
        self.Kp = Kp
        self.Kd = Kd
        self.Kt = Kt
        self.e_prev = 0.0

    def step(self, e, theta, dt):
        """
        Calculate control signal.
        
        Args:
            e: Normalized lateral error (-1 to 1)
            theta: Heading angle error (radians)
            dt: Time step (seconds)
            
        Returns:
            u: Control signal (steering angle in radians)
        """
        de = (e - self.e_prev) / dt if dt > 0 else 0.0
        self.e_prev = e
        u = self.Kp * e + self.Kd * de + self.Kt * theta
        return u


# ===================================== IMAGE PROCESSING FUNCTIONS =====================================
def get_largest_detection(result, min_conf=0.25,min_area=100):
    best = None
    best_area = 0.0

    for b in result.boxes:
        conf = float(b.conf[0])
        if conf < min_conf:
            continue

        x1, y1, x2, y2 = b.xyxy[0].tolist()
        area = max(0.0, (x2 - x1)) * max(0.0, (y2 - y1))

        if area < min_area:
            continue

        if area > best_area:
            cls = int(b.cls[0])
            best = {
                "label": result.names[cls],
                "conf": conf,
                "xyxy": (x1, y1, x2, y2),
                "area": area
            }
            best_area = area

    return best

def stable_label(history, k=3):
    """
    vraća label ako se pojavio bar k puta u poslednjih N,
    inače None
    """
    c = Counter([x for x in history if x is not None])
    if not c:
        return None
    label, cnt = c.most_common(1)[0]
    return label if cnt >= k else None


def stopLine(img,askewTreshold=40,lengthTreshold=160):
    lines=hough_transform(img)
    if lines is None:
        return False
    for line in lines:
        x1,y1,x2,y2=line.reshape(4)
        if abs(y1-y2)<askewTreshold and ((x1-x2)**2+(y1-y2)**2)**0.5 > lengthTreshold:
            return True
    return False

def warping(image,debug_bgr=None):
    """
    Perspective transform to bird's eye view.
    
    Args:
        image: Input image (grayscale or BGR)
        
    Returns:
        bev: Bird's eye view image
    """
    h, w = image.shape[:2]
    src = np.float32([
        [0.15*w, 0.25*h],   # TL
        [0.88*w, 0.25*h],   # TR
        [1*w, 0.8*h],     # BR
        [0*w, 0.8*h],       # BL
    ])
    m = int(0.18*w)  # margin
    dst = np.float32([
        [m, 0],       # TL
        [w-m, 0],     # TR
        [w-m, h],     # BR
        [m, h],       # BL
    ])
    Hm = cv2.getPerspectiveTransform(src, dst)
    bev = cv2.warpPerspective(image, Hm, (w, h),
                              flags=cv2.INTER_NEAREST,
                              borderMode=cv2.BORDER_CONSTANT,
                              borderValue=0)
    dbg = None
    if debug_bgr is not None:
        dbg = debug_bgr.copy()
        cv2.polylines(dbg, [src.astype(np.int32)], True, (0,255,0), 2)
    return bev,dbg


def hough_transform(image):
    """
    Hough line detection on binary image.
    
    Args:
        image: Binary input image
        
    Returns:
        lines: Detected line segments (Nx1x4 array)
    """
    rho = 1
    theta = np.pi/180
    threshold = 80
    minLineLength = 10
    maxLineGap = 5
    return cv2.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold,
                           minLineLength=minLineLength, maxLineGap=maxLineGap)


def preProcessing(img):
    """
    Preprocess image for lane detection.
    
    Args:
        img: BGR input image
        
    Returns:
        binary: Binary edge image
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 1)
    sobelx = cv2.Sobel(blur, cv2.CV_16S, 0, 1, ksize=3)
    sobelx = np.abs(sobelx)
    _, binary = cv2.threshold(sobelx, 55, 255, cv2.THRESH_BINARY)  #STELUJE SE
    binary = binary.astype(np.uint8)
    return binary


def bev_postprocess(bev):
    """
    Morphological preprocessing on bird's eye view.
    
    Args:
        bev: Bird's eye view binary image
        
    Returns:
        bev: Processed image with connected segments
    """
    bev = cv2.morphologyEx(bev, cv2.MORPH_CLOSE,
                           cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)),
                           iterations=2)
    return bev


def fitline_weighted_by_length(lines, dashedLength=40, k=0.05, n_min=2, n_max=25):
    """
    Fit line weighted by segment length with dashed line detection.
    
    Args:
        lines: Hough line segments
        dashedLength: Threshold for dashed line detection
        k: Sampling rate (samples per pixel)
        n_min: Minimum samples per segment
        n_max: Maximum samples per segment
        
    Returns:
        tuple: (vx, vy, x0, y0, isDashed) or None if no valid lines
    """
    if lines is None:
        return None

    lines = np.squeeze(lines)
    if lines.ndim == 1:
        lines = lines.reshape(1, 4)

    pts = []
    cntLine = 0
    totalLength = 0
    isDashed = False
    
    for x1, y1, x2, y2 in lines:
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        if abs(dy) < 5:
            continue

        L = (dx*dx + dy*dy) ** 0.5
        totalLength += L
        cntLine += 1

        n = int(np.clip(np.ceil(k * L), n_min, n_max))
        for i in range(n):
            a = i / (n - 1) if n > 1 else 0.0
            x = x1 + a * (x2 - x1)
            y = y1 + a * (y2 - y1)
            pts.append([x, y])

    if len(pts) < 2:
        return None

    points = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)
    vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_HUBER, 0, 0.01, 0.01)

    if cntLine != 0:
        if totalLength/cntLine < dashedLength:
            isDashed = True

    return float(vx[0]), float(vy[0]), float(x0[0]), float(y0[0]), isDashed


def debugSetup():
    """Standalone preview loop for testing lane preprocessing from this file."""
    try:
        camera = picamera2.Picamera2()
    except RuntimeError as e:
        msg = str(e)
        if "Device or resource busy" in msg or "did not complete" in msg:
            print("[LaneDetection] Camera is busy (already used by another process).")
            print("[LaneDetection] Stop processCamera/threadCamera (or any camera app) and try again.")
            return
        raise

    config = camera.create_preview_configuration(
        buffer_count=1,
        queue=False,
        main={"format": "RGB888", "size": (640, 480)},
    )
    camera.configure(config)
    camera.start()
    model = None
    model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best2.pt")
    try:
        if not os.path.exists(model_path):
            print(f"[LaneDetection] Model file not found: {model_path}")
        else:
            model = YOLO(model_path)
            print(f"[LaneDetection] YOLO loaded: {model_path}")
    except Exception as e:
        print(f"[LaneDetection] YOLO load failed: {e}")
        model = None

    try:
        while True:
            frame = camera.capture_array()
            if frame is None:
                continue

            # Picamera2 gives RGB888 here; lane pipeline expects BGR.
            #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            lane_roi = frame[240:480, :]
            lane_img = cv2.resize(lane_roi, (320, 240), interpolation=cv2.INTER_AREA)

            binary = preProcessing(lane_img)
            warped, dbg = warping(binary, lane_img)
            #warped = bev_postprocess(warped)

            cv2.imshow("Lane Input", lane_img)
            cv2.imshow("Lane Binary", binary)
            cv2.imshow("Lane Warped", warped)
            if dbg is not None:
                cv2.imshow("Lane ROI", dbg)
            results = model(frame, imgsz=320, conf=0.2)
            # annotated = results[0].plot()
            # cv2.imshow("detections", annotated)
            # cv2.moveWindow("detections", 200, 200)
            # cv2.waitKey(1)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()


def saturate(u, u_max):
    """Saturate control signal to [-u_max, u_max]."""
    return max(-u_max, min(u, u_max))


# ===================================== LANE DETECTION CLASS =====================================
class LaneDetection:
    """
    Lane Detection class that receives frames from central camera process.
    
    This class encapsulates all lane detection functionality:
    - Subscribes to laneCamera message queue for frames
    - Image processing and lane detection
    - Steering angle calculation
    
    NOTE: This class does NOT initialize its own camera! It relies on
    processCamera/threadCamera to be running and sending frames via
    the laneCamera message queue.
    """
    
    def __init__(self, queueList, logger, debugging=False):
        """
        Initialize the lane detection system.
        
        Args:
            queueList: Dictionary of message queues for receiving camera frames
            logger: Logger object for debugging output
            debugging: Enable debug logging
        """
        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging
        
        # Message subscriber for camera frames
        self.cameraSubscriber = None
        self.is_running = False
        
        # Image dimensions (from laneCamera stream: 640x480)
        self.H = 480
        self.W = 640
        self.xc=None
        
        # Lane detection state
        self.prev_angle_left = 0
        self.prev_angle_right = 0
        self.prev_x_left = self.W * 0.1
        self.prev_x_right = self.W * 0.9
        self.laneWidth = self.W * 0.8
        self.is_stop_line = False

        #ai detection
        self.model = None
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best2.pt")
        try:
            if not os.path.exists(model_path):
                self.logger.error(f"[LaneDetection] Model file not found: {model_path}")
            else:
                self.model = YOLO(model_path)
                self.logger.info(f"[LaneDetection] YOLO loaded: {model_path}")
        except Exception as e:
            self.logger.error(f"[LaneDetection] YOLO load failed: {e}")
            self.model = None
        self.history = deque(maxlen=5)
        self.current = None
        self.hold = 0
        self.HOLD_FRAMES = 4
        self.sign_period = 1
        self.last_sign_t = 0.0
        
        # Controller (moderate gains for stable lane keeping)
        # Kt=3.5 balances lane centering with turn response
        self.controller = LaneController(Kp=8, Kd=1, Kt=3)
        self.t_prev = time.time()
        self.last_steering = 0
        
        # Frame buffer
        self.last_frame = None
        
        self.logger.info("[LaneDetection] Initialized (using central camera)")

    def start(self):
        """Start the lane detection system (subscribe to camera frames)."""
        if self.is_running:
            return True
            
        self.logger.info("[LaneDetection] Starting (subscribing to laneCamera)...")
        try:
            # Subscribe to laneCamera messages from threadCamera
            self.cameraSubscriber = messageHandlerSubscriber(
                self.queueList, laneCamera, "lastOnly", True
            )
            self.is_running = True
            self.t_prev = time.time()
            self.logger.info("[LaneDetection] ✓ Subscribed to laneCamera stream")
            return True
        except Exception as e:
            self.logger.error(f"[LaneDetection] Subscription failed: {e}")
            return False

    def stop(self):
        """Stop the lane detection system."""
        if not self.is_running:
            return
            
        self.logger.info("[LaneDetection] Stopping...")
        # Properly unsubscribe from the gateway before releasing the subscriber
        if self.cameraSubscriber is not None:
            try:
                self.cameraSubscriber.unsubscribe()
            except Exception as e:
                self.logger.warning(f"[LaneDetection] Unsubscribe warning: {e}")
        self.cameraSubscriber = None
        self.is_running = False
        self.last_frame = None
        self.logger.info("[LaneDetection] ✓ Stopped")

    def _get_frame(self):
        """
        Get the latest frame from the camera subscriber.
        
        Returns:
            numpy array: BGR frame (640x480) or None if no frame available
        """
        if self.cameraSubscriber is None:
            return None
            
        try:
            frameData = self.cameraSubscriber.receive()
            if frameData is None:
                return self.last_frame  # Return cached frame
            
            # Reconstruct numpy array from bytes
            shape = frameData["shape"]
            dtype = np.dtype(frameData["dtype"])
            data = frameData["data"]
            
            frame = np.frombuffer(data, dtype=dtype).reshape(shape)
            self.last_frame = frame
            return frame
            
        except Exception as e:
            if self.debugging:
                self.logger.error(f"[LaneDetection] Frame receive error: {e}")
            return self.last_frame

    def process_frame(self, frame):
        """
        Process a single frame for lane detection.
        
        Args:
            frame: BGR input frame (640x480)
            
        Returns:
            tuple: (steering_degrees, success)
        """
        try:
            # Extract lane ROI (bottom half) and resize
            lane_roi = frame[self.H//2:self.H, :]
            lane_img = cv2.resize(lane_roi, (320, 240), interpolation=cv2.INTER_AREA)
            
            # Preprocess
            binary = preProcessing(lane_img)
            
            # Warp to bird's eye view
            warped_lane, _ = warping(binary)
            warped_lane = bev_postprocess(warped_lane)
            
            h_warp, w_warp = warped_lane.shape[:2]
            
            # Hough transform on left and right halves
            lines_left = hough_transform(warped_lane[:, :w_warp//2])
            lines_right = hough_transform(warped_lane[:, w_warp//2:])
            
            # Offset right lines
            if lines_right is not None:
                lines_right[:, :, 0] += w_warp//2
                lines_right[:, :, 2] += w_warp//2
            
            # Avoid bottom-border artifacts: inspect a band in front of the car
            # rather than the last few pixels at the image edge.
            
            self.is_stop_line = stopLine(warped_lane[200:240,])
            

            # Fit left line
            angle_left = None
            x_left = None
            res = fitline_weighted_by_length(lines_left, 70)
            if res is not None:
                vx, vy, x0, y0, isDashed = res
                angle_left = np.arctan2(vx, vy)
                angle_left *= -1
                if angle_left > np.pi/2:
                    angle_left -= np.pi
                elif angle_left < -np.pi/2:
                    angle_left += np.pi
                
                x_left = x0 + (h_warp*0.75 - y0) * (vx / vy)
                
                # Filter only extreme jumps (allow real lane changes)
                if abs(angle_left - self.prev_angle_left) > 0.175:
                    angle_left = self.prev_angle_left
                if abs(x_left - self.prev_x_left) > 50:
                    x_left = self.prev_x_left
            
            # Fit right line
            angle_right = None
            x_right = None
            res = fitline_weighted_by_length(lines_right, 70)
            if res is not None:
                vx, vy, x0, y0, isDashed = res
                angle_right = np.arctan2(vx, vy)
                angle_right *= -1
                if angle_right > np.pi/2:
                    angle_right -= np.pi
                elif angle_right < -np.pi/2:
                    angle_right += np.pi
                
                x_right = x0 + (h_warp*0.75 - y0) * (vx / vy)
                
                # Filter only extreme jumps (allow real lane changes)
                if abs(angle_right - self.prev_angle_right) > 0.175:
                    angle_right = self.prev_angle_right
                if abs(x_right - self.prev_x_right) > 50:
                    x_right = self.prev_x_right
            if not self.xc:
                if x_right and x_left:
                    self.xc=(x_right+x_left)/2
                else:
                    self.xc=self.W/2
            # Calculate error and angle
            xc=self.xc
            e = 0
            theta = 0
            
            if x_left is None and x_right is None:
                # No lines detected - keep going straight with decay
                e=(self.prev_x_left+self.prev_x_right)/2-xc  
                theta=(self.prev_angle_left+self.prev_angle_right)/2
            elif x_left is None and x_right is not None:
                e = (x_right - self.laneWidth/2) - xc
                theta = angle_right
                self.prev_angle_right = angle_right
                self.prev_x_right = x_right
            elif x_left is not None and x_right is None:
                e = (x_left + self.laneWidth/2) - xc
                theta = angle_left
                self.prev_x_left = x_left
                self.prev_angle_left = angle_left
            else:
                e = (x_left + x_right) / 2 - xc
                theta = (angle_left + angle_right) / 2
                self.prev_angle_left = angle_left
                self.prev_angle_right = angle_right
                self.prev_x_right = x_right
                self.prev_x_left = x_left
            
            # Normalize error (invert sign for correct steering direction)
            en = e / (self.W / 2)
            
            # Calculate control signal
            t = time.time()
            dt = t - self.t_prev
            self.t_prev = t
            
            u = self.controller.step(en, theta, dt)
            #u = u - 0.08  # Smaller left offset to avoid overcorrecting left
            u = saturate(u, np.deg2rad(25))
            
            # Convert to degrees (scaled for serial protocol)
            # Must be int, not float - Serial Handler expects int
            deg = int(round(u * 573, -1))  # *57.3 for rad to deg, *10 for scaling
            self.last_steering = deg
            label_now=None
            now = time.time()
            if now - self.last_sign_t >= self.sign_period:
                self.last_sign_t = now
                sign_roi = frame[0:self.H//2, self.W//2:self.W]            # 640x288
                results = self.model(sign_roi, imgsz=320, conf=0.5)

                det=get_largest_detection(results[0],min_conf=0.4)
                label_now = det["label"] if det else None

                self.history.append(label_now)

                label_stable = stable_label(self.history, k=3)


                # if label_stable is not None:

                #     self.current = label_stable
                #     self.hold = self.HOLD_FRAMES

                # else:

                #     if self.hold > 0:
                #         self.hold -= 1
                #     else:
                #         self.current = None
            
            return deg,label_now,self.is_stop_line
            
        except Exception as e:
            print(f"[LaneDetection] Frame processing error: {e}")
            return self.last_steering, None, self.is_stop_line

    def get_drive_params(self):
        """
        Get the current drive parameters based on lane detection.
        
        Returns:
            tuple: (steering_angle, sign, stopline)
        """
        if not self.is_running:
            return None
            
        frame = self._get_frame()
        if frame is None:
            return self.last_steering,None,None
            
        steering, sign, stopline = self.process_frame(frame)
        return steering, sign, stopline

    def is_active(self):
        """Check if lane detection is currently active."""
        return self.is_running

    def drain_frame(self):
        """
        Read and discard latest frame to prevent laneCamera queue buildup.
        """
        _ = self._get_frame()
        return True

if __name__ == "__main__":
    debugSetup()