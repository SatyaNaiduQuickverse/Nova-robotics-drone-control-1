"""Chase — drone object tracker. YOLO (Hailo-8) + Kalman + multi-cue re-ID."""
import cv2
import json
import os
import time
import sys
import threading
import numpy as np
import urllib.request
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
from hailo_platform import (
    VDevice, HEF, ConfigureParams, InferVStreams,
    InputVStreamParams, OutputVStreamParams,
    HailoStreamInterface, FormatType,
)

HOST = "0.0.0.0"
PORT = int(os.environ.get("VISION_PORT", "8081"))

# --- SIM SOURCE (remove this block + all SIM_SOURCE refs when done) ---
# Usage: python3 detector.py --source sim
# Reads frames from Gazebo camera bridge instead of USB webcam.
SIM_SOURCE = "--source" in sys.argv and "sim" in sys.argv
SIM_URL = "http://127.0.0.1:8082/frame"
if SIM_SOURCE:
    print("*** SIM MODE: reading from Gazebo camera bridge ***")
# --- END SIM SOURCE ---

# --- DRONE SOURCE ---
# Usage: python3 detector.py --source drone   (or VISION_SOURCE=drone)
# Pulls frames from drone-control's /camera/snapshot over the host network
# so we don't fight camera.py for /dev/video0. drone-control stays the
# single v4l2 owner; the vision container is a downstream consumer.
DRONE_SOURCE = (
    ("--source" in sys.argv and "drone" in sys.argv)
    or os.environ.get("VISION_SOURCE") == "drone"
)
DRONE_URL = os.environ.get("VISION_DRONE_URL", "http://127.0.0.1:8080/camera/tracking/snapshot")
if DRONE_SOURCE:
    print(f"*** DRONE MODE: reading from {DRONE_URL} ***")
# --- END DRONE SOURCE ---

_state = {"jpg": None, "dets": [], "seq": 0, "tracking": False, "lost": False}
_state_lock = threading.Lock()
_lock_req = None
_lock_req_lock = threading.Lock()

# Latest raw BGR frame from the grabber thread — single slot, no queue.
# The inference thread always pulls the freshest frame and drops everything
# older. This is load-bearing for drone safety: if YOLO spends 150ms on a
# frame, we don't want the next inference pass to chew through a 150ms-old
# backlog while real-world state has moved on.
_latest_frame = None           # np.ndarray (BGR) or None
_latest_seq = 0                # monotonically increases per grabbed frame
_latest_ts = 0.0               # time.monotonic() at the moment of grab; used
                               # by /state to publish frame_age_ms (drone-side
                               # vision-lock contract rule 2b heartbeat).
_latest_lock = threading.Lock()

HEF_PATH = os.environ.get("VISION_HEF", "/models/yolov8s_h8.hef")
MODEL_INPUT = int(os.environ.get("VISION_IMGSZ", "640"))  # fixed by HEF; yolov8s_h8 = 640
CONF = float(os.environ.get("VISION_CONF", "0.25"))

# COCO 80 class names — inlined so we don't need ultralytics at runtime.
COCO_NAMES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush",
]


def _draw_chip(frame, text, x, y, fg):
    """Small rounded-rect label chip. Used for FPS + status overlays."""
    f, scale, th = cv2.FONT_HERSHEY_DUPLEX, 0.6, 1
    (tw, th_px), baseline = cv2.getTextSize(text, f, scale, th)
    pad_x, pad_y = 10, 7
    x2 = x + tw + 2 * pad_x
    y2 = y + th_px + 2 * pad_y + baseline
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    cv2.rectangle(frame, (x, y), (x2, y2), (70, 70, 70), 1, cv2.LINE_AA)
    cv2.putText(frame, text, (x + pad_x, y + pad_y + th_px),
                f, scale, fg, th, cv2.LINE_AA)


def _draw_label(frame, text, x, y, fg):
    """Compact label anchored above a box's top-left corner."""
    f, scale, th = cv2.FONT_HERSHEY_DUPLEX, 0.5, 1
    (tw, th_px), baseline = cv2.getTextSize(text, f, scale, th)
    pad = 6
    ly2 = y - 4
    ly1 = ly2 - th_px - 2 * pad
    lx2 = x + tw + 2 * pad
    if ly1 < 0:
        ly1, ly2 = y + 4, y + th_px + 2 * pad + 4
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, ly1), (lx2, ly2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, text, (x + pad, ly2 - pad - 1),
                f, scale, fg, th, cv2.LINE_AA)


def _draw_reticle(frame, box, color, label):
    """HUD-style corner-bracket reticle with crosshair at the center.
    Scales with both the box size and the frame size so it reads clearly
    at any capture resolution (480p, 720p, 1080p)."""
    x1, y1, x2, y2 = [int(v) for v in box]
    bw, bh = x2 - x1, y2 - y1
    fh, fw = frame.shape[:2]
    # Bracket length: proportional to box, capped by a fraction of frame.
    L = max(10, min(bw // 3, bh // 3, int(min(fw, fh) * 0.06)))
    # Stroke thickness scales with frame size — 2 px at 720p+, 1 px at 480p.
    t = 2 if min(fw, fh) >= 600 else 1
    for (cx, cy, sx, sy) in (
        (x1, y1,  1,  1),
        (x2, y1, -1,  1),
        (x1, y2,  1, -1),
        (x2, y2, -1, -1),
    ):
        cv2.line(frame, (cx, cy), (cx + sx * L, cy), color, t, cv2.LINE_AA)
        cv2.line(frame, (cx, cy), (cx, cy + sy * L), color, t, cv2.LINE_AA)
    # Center crosshair
    cx_c, cy_c = (x1 + x2) // 2, (y1 + y2) // 2
    arm = max(8, int(min(fw, fh) * 0.018))
    gap = max(4, arm // 3)
    cv2.line(frame, (cx_c - arm, cy_c), (cx_c - gap, cy_c), color, t, cv2.LINE_AA)
    cv2.line(frame, (cx_c + gap, cy_c), (cx_c + arm, cy_c), color, t, cv2.LINE_AA)
    cv2.line(frame, (cx_c, cy_c - arm), (cx_c, cy_c - gap), color, t, cv2.LINE_AA)
    cv2.line(frame, (cx_c, cy_c + gap), (cx_c, cy_c + arm), color, t, cv2.LINE_AA)
    if label:
        _draw_label(frame, label, x1, y1, color)


class HailoYolo:
    """Hailo-8 YOLOv8 wrapper. On-chip NMS. Returns dets in ORIGINAL frame coords.

    Thread model: the Hailo VDevice, configured network group, and InferVStreams
    context are all held open for the process lifetime. `predict()` is called
    from a single inference thread (capture_loop), so no extra locking is needed.
    """

    def __init__(self, hef_path: str, conf: float = 0.25):
        self.conf = conf
        self.input_size = MODEL_INPUT
        self.names = COCO_NAMES

        self._hef = HEF(hef_path)
        self._vdevice = VDevice()
        cfg = ConfigureParams.create_from_hef(
            self._hef, interface=HailoStreamInterface.PCIe
        )
        self._ng = self._vdevice.configure(self._hef, cfg)[0]
        self._in_params = InputVStreamParams.make(self._ng, format_type=FormatType.UINT8)
        self._out_params = OutputVStreamParams.make(self._ng, format_type=FormatType.FLOAT32)
        self._input_name = list(self._in_params.keys())[0]

        self._ng_params = self._ng.create_params()
        self._activation = self._ng.activate(self._ng_params)
        self._activation.__enter__()
        self._pipe = InferVStreams(self._ng, self._in_params, self._out_params)
        self._pipe.__enter__()

    def _letterbox(self, frame):
        h, w = frame.shape[:2]
        s = min(self.input_size / w, self.input_size / h)
        nw, nh = int(round(w * s)), int(round(h * s))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        top = (self.input_size - nh) // 2
        left = (self.input_size - nw) // 2
        canvas[top:top + nh, left:left + nw] = resized
        return canvas, s, left, top

    def predict(self, frame):
        """Run one inference. Returns list of {"cls","conf","box"[x1,y1,x2,y2]}
        in ORIGINAL frame pixel coordinates."""
        canvas, s, pad_x, pad_y = self._letterbox(frame)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        inp = rgb[None, ...]  # NHWC batch=1

        out = self._pipe.infer({self._input_name: inp})
        # HAILO NMS BY CLASS: dict -> single key -> [batch][class_idx] = Nx5
        # where each row is [y1, x1, y2, x2, score] normalized to [0,1].
        raw = list(out.values())[0][0]

        fh, fw = frame.shape[:2]
        isz = self.input_size
        dets = []
        for class_idx, boxes in enumerate(raw):
            if boxes is None or len(boxes) == 0:
                continue
            for row in boxes:
                score = float(row[4])
                if score < self.conf:
                    continue
                # Normalized in letterboxed 640x640 → pixels → un-pad → un-scale
                y1 = float(row[0]) * isz
                x1 = float(row[1]) * isz
                y2 = float(row[2]) * isz
                x2 = float(row[3]) * isz
                x1 = max(0, min(fw - 1, (x1 - pad_x) / s))
                y1 = max(0, min(fh - 1, (y1 - pad_y) / s))
                x2 = max(0, min(fw - 1, (x2 - pad_x) / s))
                y2 = max(0, min(fh - 1, (y2 - pad_y) / s))
                if x2 <= x1 or y2 <= y1:
                    continue
                name = self.names[class_idx] if class_idx < len(self.names) else str(class_idx)
                dets.append({
                    "cls": name,
                    "conf": score,
                    "box": [int(x1), int(y1), int(x2), int(y2)],
                })
        return dets


class Target:
    """Tracks a single object using Kalman filter + histogram re-ID."""

    def __init__(self, frame, box_xyxy, cls=None):
        x1, y1, x2, y2 = box_xyxy
        self.cls = cls
        self.w = x2 - x1
        self.h = y2 - y1

        # Kalman: state = [cx, cy, vx, vy], measurement = [cx, cy]
        # High process noise → can follow fast, sudden movements
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        pn = np.eye(4, dtype=np.float32)
        pn[0,0] = pn[1,1] = 10.0   # position noise
        pn[2,2] = pn[3,3] = 50.0   # velocity noise — high to track fast changes
        self.kf.processNoiseCov = pn
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 10.0
        cx, cy = (x1+x2)/2, (y1+y2)/2
        self.kf.statePost = np.array([[cx],[cy],[0],[0]], np.float32)

        # Appearance: HSV histogram (frozen original + adaptive)
        self.orig_hist = self._hist(frame, box_xyxy)
        self.run_hist = self.orig_hist.copy() if self.orig_hist is not None else None

        # Correlation-filter tracker for cls=None (non-COCO) locks. KCF is
        # the primary tracker for these — its .update() both predicts and
        # learns, which is what gives continuous adaptation to pose/lighting
        # changes. But naive use corrupts the model during occlusion: if the
        # target is blocked (hand, tree), .update() trains on the occluder,
        # and when the occluder leaves the filter has forgotten the real
        # target. We defend against this by snapshotting the last-known-good
        # frame + box on every valid update, and REWINDING KCF from that
        # snapshot (cheap 0.65ms re-init) whenever the current update fails
        # size/histogram gates. During a long occlusion, KCF is rewound each
        # frame → stays frozen on the last good state → re-locks on the real
        # target the moment the scene clears.
        self.kcf = None
        self.last_good_frame = None
        self.last_good_box = None
        # Consecutive-frame drift confirmation. A single bad frame from
        # motion blur or a lighting flicker shouldn't trigger rewind —
        # only persistent drift does. Requiring 2+ bad frames in a row
        # is enough: real occlusion lasts hundreds of frames, transient
        # glitches last 1-2.
        self.drift_streak = 0
        if cls is None:
            try:
                self.kcf = cv2.TrackerKCF_create()
                self.kcf.init(frame, (int(x1), int(y1), int(self.w), int(self.h)))
                self.last_good_frame = frame.copy()
                self.last_good_box = [int(x1), int(y1), int(x2), int(y2)]
            except Exception as e:
                print(f"KCF init failed: {e}")
                self.kcf = None

        self.lost_frames = 0
        self.total_frames = 0
        self.last_box = box_xyxy

    def _hist(self, frame, box_xyxy):
        x1, y1, x2, y2 = [int(v) for v in box_xyxy]
        # Use inner 70% to avoid background
        pw, ph = int((x2-x1)*0.15), int((y2-y1)*0.15)
        crop = frame[max(0,y1+ph):y2-ph, max(0,x1+pw):x2-pw]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist)
        return hist

    def predict(self):
        """Kalman predict → returns predicted center (cx, cy)."""
        p = self.kf.predict()
        return float(p[0, 0]), float(p[1, 0])

    def update_found(self, frame, box_xyxy):
        """Object found — update Kalman, histogram, size."""
        x1, y1, x2, y2 = box_xyxy
        cx, cy = (x1+x2)/2, (y1+y2)/2
        self.kf.correct(np.array([[cx],[cy]], np.float32))
        self.w = 0.8 * self.w + 0.2 * (x2 - x1)  # smooth size
        self.h = 0.8 * self.h + 0.2 * (y2 - y1)
        self.last_box = box_xyxy
        self.lost_frames = 0

        # Update running histogram
        new_hist = self._hist(frame, box_xyxy)
        if new_hist is not None and self.run_hist is not None:
            self.run_hist = 0.93 * self.run_hist + 0.07 * new_hist
        elif new_hist is not None:
            self.run_hist = new_hist

    def update_lost(self):
        """Object not found — just predict."""
        self.lost_frames += 1

    def _rewind_kcf(self):
        """Rebuild KCF from the last-known-good frame + box. Used when a
        bad .update() has corrupted the filter — restores it to the state
        it was in before the occluder arrived. Cheap: KCF init is 0.65ms."""
        if self.last_good_frame is None or self.last_good_box is None:
            return
        try:
            x1, y1, x2, y2 = self.last_good_box
            self.kcf = cv2.TrackerKCF_create()
            self.kcf.init(self.last_good_frame,
                          (int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        except Exception:
            pass

    def search_and_rescue(self, frame):
        """Wide-area recovery. KCF's native search radius is small (~1.5x
        the target box), so when the operator yanks the drone and the
        target jumps by more pixels than that, KCF locks onto background
        and we drift. This method does a multi-scale full-region template
        match of the last-known-good crop against a generous search area
        around the last-known position, returns the peak location if it
        scores above `min_score`, and re-inits KCF there. Called whenever
        drift is detected.

        Cost: ~8-25ms depending on search region size. Only runs on drift
        events, so average CPU impact is minimal."""
        if self.last_good_frame is None or self.last_good_box is None:
            return None
        lx1, ly1, lx2, ly2 = self.last_good_box
        tw, th = lx2 - lx1, ly2 - ly1
        if tw < 8 or th < 8:
            return None
        template = self.last_good_frame[max(0, ly1):ly2, max(0, lx1):lx2]
        if template.size == 0:
            return None

        fh, fw = frame.shape[:2]
        # Search region: 4x the target bounding box, centered on the last
        # known position. Clamped to frame bounds. 4x covers ~2x motion on
        # each side — enough to recover from a sudden drone jerk but not
        # so wide that similar bystander objects get picked up.
        cx = (lx1 + lx2) // 2
        cy = (ly1 + ly2) // 2
        sw = tw * 4
        sh = th * 4
        sx1 = max(0, cx - sw // 2)
        sy1 = max(0, cy - sh // 2)
        sx2 = min(fw, cx + sw // 2)
        sy2 = min(fh, cy + sh // 2)
        search = frame[sy1:sy2, sx1:sx2]
        if search.shape[0] < template.shape[0] or search.shape[1] < template.shape[1]:
            return None

        # Multi-scale template match: target scale can change due to
        # drone-to-target distance variation.
        best_score = -1.0
        best_loc = None
        best_scale = 1.0
        for scale in (0.8, 1.0, 1.25):
            if scale != 1.0:
                nw, nh = int(tw * scale), int(th * scale)
                if nw < 8 or nh < 8:
                    continue
                if nh > search.shape[0] or nw > search.shape[1]:
                    continue
                tmpl = cv2.resize(template, (nw, nh))
            else:
                tmpl = template
            if tmpl.shape[0] > search.shape[0] or tmpl.shape[1] > search.shape[1]:
                continue
            res = cv2.matchTemplate(search, tmpl, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)
            if max_val > best_score:
                best_score = max_val
                best_loc = max_loc
                best_scale = scale

        if best_loc is None or best_score < 0.35:
            return None

        nw = int(tw * best_scale)
        nh = int(th * best_scale)
        new_x1 = sx1 + best_loc[0]
        new_y1 = sy1 + best_loc[1]
        new_x2 = new_x1 + nw
        new_y2 = new_y1 + nh
        new_box = [new_x1, new_y1, new_x2, new_y2]

        # Re-init KCF on the recovered position IN THE CURRENT FRAME so
        # subsequent updates see the drone's new viewpoint.
        try:
            self.kcf = cv2.TrackerKCF_create()
            self.kcf.init(frame, (int(new_x1), int(new_y1), int(nw), int(nh)))
            self.last_good_frame = frame.copy()
            self.last_good_box = new_box
            self.drift_streak = 0
            print(f"[rescue] recovered score={best_score:.2f} scale={best_scale}")
        except Exception:
            return None
        return new_box

    def tracker_update(self, frame):
        """Run the correlation filter one step. On success, snapshot the
        current frame+box as last-known-good. On failure (occluder, drift),
        rewind KCF to the last good snapshot so subsequent frames don't
        train on the occluder.

        Drift detection combines three gates, any one of which rejects:
          1. Size sanity — KCF's box within 40–180% of target dimensions.
          2. Histogram correlation — HSV hist of candidate crop vs
             stored orig/run hists > 0.20 (weak signal but catches
             dramatic color changes).
          3. Pixel template match — normalized cross-correlation between
             the last-good crop and the candidate crop > 0.35. This is
             the strong signal that catches occluders whose coarse color
             histogram happens to resemble the target.
        """
        if self.kcf is None:
            return None
        ok, box = self.kcf.update(frame)
        drifted = not ok
        cand_box = None
        ncc = None
        s_orig = s_run = None
        if ok:
            x, y, w, h = [int(v) for v in box]
            if (w > self.w * 1.8 or h > self.h * 1.8 or
                    w < self.w * 0.4 or h < self.h * 0.4):
                drifted = True
            else:
                cand_box = [x, y, x + w, y + h]
                cand_hist = self._hist(frame, cand_box)
                if cand_hist is not None and self.orig_hist is not None:
                    s_orig = cv2.compareHist(self.orig_hist, cand_hist,
                                             cv2.HISTCMP_CORREL)
                    s_run = (cv2.compareHist(self.run_hist, cand_hist,
                                             cv2.HISTCMP_CORREL)
                             if self.run_hist is not None else 0)
                    # Loose threshold (0.15) tolerates pose/lighting
                    # changes during drone flight. Histogram is a weak
                    # signal — occluders are caught by the pixel NCC
                    # check below, not this one.
                    if max(s_orig, s_run) < 0.15:
                        drifted = True
                # Pixel-level template match against the last good crop.
                # This is the critical occlusion detector: a hand over
                # the lens will score near zero here no matter what its
                # coarse color histogram looks like.
                if not drifted and self.last_good_frame is not None:
                    lx1, ly1, lx2, ly2 = self.last_good_box
                    ref = self.last_good_frame[max(0, ly1):ly2,
                                               max(0, lx1):lx2]
                    fh, fw = frame.shape[:2]
                    cx1 = max(0, cand_box[0])
                    cy1 = max(0, cand_box[1])
                    cx2 = min(fw, cand_box[2])
                    cy2 = min(fh, cand_box[3])
                    cur = frame[cy1:cy2, cx1:cx2]
                    if ref.size > 0 and cur.size > 0:
                        if cur.shape[:2] != ref.shape[:2]:
                            cur = cv2.resize(cur, (ref.shape[1],
                                                   ref.shape[0]))
                        res = cv2.matchTemplate(cur, ref,
                                                cv2.TM_CCOEFF_NORMED)
                        ncc = float(res[0, 0])
                        # 0.15 tolerates motion blur + lighting shifts
                        # while still rejecting a hand-over-camera
                        # (which scores near 0 — totally uncorrelated
                        # structure with the target).
                        if ncc < 0.15:
                            drifted = True

        if drifted:
            self.drift_streak += 1
            # Require two consecutive bad frames before rewinding. A
            # single bad frame is almost always a motion-blur glitch;
            # real occlusion holds for many frames.
            if self.drift_streak >= 2:
                print(f"[kcf] drift confirmed streak={self.drift_streak} "
                      f"s_orig={s_orig} s_run={s_run} ncc={ncc}")
                self._rewind_kcf()
                return None
            # First bad frame: don't return a candidate (caller treats
            # as lost for this frame) but don't corrupt KCF state either.
            # We leave KCF as-is and hope the next frame recovers.
            return None
        self.drift_streak = 0

        # Valid step — snapshot it as the new last-known-good. Frame copy
        # is ~1ms for 640x480; we only pay this on successful frames so
        # occlusion periods cost nothing extra.
        self.last_good_frame = frame.copy()
        self.last_good_box = list(cand_box)
        return cand_box

    def match_score(self, frame, box_xyxy, reacquire=False):
        """Score a candidate. Two modes:
        - Tracking: distance is king (object can't teleport), histogram confirms
        - Re-acquire: histogram is king (object could be anywhere), distance is soft
        """
        x1, y1, x2, y2 = box_xyxy
        cx, cy = (x1+x2)/2, (y1+y2)/2
        pred_cx, pred_cy = self.predict_pos()

        dist = np.sqrt((cx - pred_cx)**2 + (cy - pred_cy)**2)

        # Size score
        cw, ch = max(1, x2-x1), max(1, y2-y1)
        size_score = min(cw/(self.w+1), (self.w+1)/cw) * min(ch/(self.h+1), (self.h+1)/ch)

        # Histogram score
        cand_hist = self._hist(frame, box_xyxy)
        hist_score = 0
        if cand_hist is not None:
            s1 = cv2.compareHist(self.orig_hist, cand_hist, cv2.HISTCMP_CORREL) if self.orig_hist is not None else 0
            s2 = cv2.compareHist(self.run_hist, cand_hist, cv2.HISTCMP_CORREL) if self.run_hist is not None else 0
            hist_score = max(s1, s2)

        if reacquire:
            # Re-acquire: histogram dominates, distance very forgiving
            dist_score = np.exp(-dist**2 / (2 * 400**2))
            return 0.60 * hist_score + 0.15 * dist_score + 0.25 * size_score
        else:
            # Tracking: must look like the target AND be near it
            # Hard gate: wrong color = instant reject, no matter how close
            if hist_score < 0.20:
                return 0.0
            dist_score = np.exp(-dist**2 / (2 * 120**2))
            return 0.30 * hist_score + 0.50 * dist_score + 0.20 * size_score

    def predict_pos(self):
        """Get current predicted position without advancing state."""
        s = self.kf.statePost
        return float(s[0, 0]), float(s[1, 0])

    def predicted_box(self):
        """Get predicted box as [x1, y1, x2, y2]."""
        cx, cy = self.predict_pos()
        hw, hh = self.w/2, self.h/2
        return [int(cx-hw), int(cy-hh), int(cx+hw), int(cy+hh)]

def grabber_loop():
    """
    Dedicated frame-grabbing thread. Pushes the freshest decoded BGR frame
    into the single-slot _latest_frame. If the inference loop is slow, old
    frames are dropped (never queued) — this is essential so the tracker
    always sees ~now, not a backlog.
    """
    global _latest_frame, _latest_seq, _latest_ts

    cap = None
    if SIM_SOURCE:
        print(f"Sim source: {SIM_URL}")
    elif DRONE_SOURCE:
        print(f"Drone source: {DRONE_URL}")
    else:
        for i in range(4):
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                print(f"Camera at /dev/video{i}"); break
            cap.release(); cap = None
        if not cap or not cap.isOpened():
            print("No camera found"); return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # drop stale on read()
        print(f"Capture: {int(cap.get(3))}x{int(cap.get(4))} @ {int(cap.get(5))}fps")

    while True:
        frame = None
        if SIM_SOURCE:
            try:
                jpg_data = urllib.request.urlopen(SIM_URL, timeout=1).read()
                frame = cv2.imdecode(np.frombuffer(jpg_data, np.uint8), cv2.IMREAD_COLOR)
            except Exception:
                time.sleep(0.03)
                continue
        elif DRONE_SOURCE:
            try:
                jpg_data = urllib.request.urlopen(DRONE_URL, timeout=1).read()
                frame = cv2.imdecode(np.frombuffer(jpg_data, np.uint8), cv2.IMREAD_COLOR)
            except Exception:
                time.sleep(0.05)
                continue
        else:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue

        if frame is None:
            continue

        with _latest_lock:
            _latest_frame = frame
            _latest_seq += 1
            _latest_ts = time.monotonic()

        # Rate-limit the grabber to ~30 Hz on the success path. Without this,
        # urlopen + imdecode loop runs at 60-100 Hz against drone-control's
        # /camera/tracking/snapshot which only produces new frames at ~25 Hz —
        # we'd decode the same JPEG 2-4 times before a new one appears,
        # burning ~30% of one CPU core on duplicate decodes. Cap to camera-rate.
        # Error paths above (sim/drone HTTP failure, V4L2 read failure) keep
        # their own backoffs; this sleep applies only after a successful grab.
        if SIM_SOURCE or DRONE_SOURCE:
            time.sleep(0.033)


def capture_loop():
    """
    Inference + tracking loop. Pulls the LATEST frame from _latest_frame on
    every iteration, drops everything older. Frame reads are cheap; inference
    is the bottleneck — this design guarantees the tracker always works off
    the freshest possible input.
    """
    global _lock_req

    yolo = HailoYolo(HEF_PATH, conf=CONF)
    print(f"Hailo YOLO loaded: {HEF_PATH} (input={MODEL_INPUT})")

    # Warmup
    try:
        yolo.predict(np.zeros((MODEL_INPUT, MODEL_INPUT, 3), dtype=np.uint8))
    except Exception as e:
        print(f"Warmup failed: {e}")

    target = None  # type: Target | None
    last_seq = 0
    fps_t = time.time()
    fps_count = 0
    shown_fps = 0

    while True:
        # Grab the freshest available frame; skip if nothing new (tight spin).
        with _latest_lock:
            if _latest_frame is None or _latest_seq == last_seq:
                frame = None
            else:
                frame = _latest_frame  # no copy — grabber writes a new ndarray
                last_seq = _latest_seq
        if frame is None:
            time.sleep(0.003)
            continue

        # Handle lock/unlock
        with _lock_req_lock:
            req = _lock_req
            _lock_req = None
        if req is not None:
            if req.get("unlock"):
                target = None
                print("Unlocked")
            elif "box" in req:
                bx, by, bw, bh = [int(v) for v in req["box"]]
                target = Target(frame, [bx, by, bx+bw, by+bh], cls=req.get("cls"))
                print(f"Locked: cls={req.get('cls')} box=({bx},{by},{bw},{bh})")

        # Run YOLO on Hailo-8 — on-chip NMS, ~10ms at 640x640.
        dets = yolo.predict(frame)

        # Track target
        locked_box = None
        is_lost = False

        if target is not None:
            target.total_frames += 1

            # Kalman predict
            target.predict()

            # Try to match among YOLO detections
            is_reacquiring = target.lost_frames > 2
            min_score = 0.40 if is_reacquiring else 0.35
            best_score = min_score
            best_det = None
            for d in dets:
                if target.cls and d["cls"] != target.cls:
                    continue
                dw = d["box"][2] - d["box"][0]
                dh = d["box"][3] - d["box"][1]
                if dw > target.w * 3 or dh > target.h * 3:
                    continue
                if dw < target.w * 0.2 or dh < target.h * 0.2:
                    continue
                score = target.match_score(frame, d["box"], reacquire=is_reacquiring)
                if score > best_score:
                    best_score = score
                    best_det = d

            # For cls=None drag-selects, KCF is the PRIMARY tracker. YOLO
            # usually has nothing useful (non-COCO objects), so we run KCF
            # every frame — its .update() both predicts AND learns, which is
            # what lets the lock adapt to appearance/lighting/scale changes
            # over the 30+ second tracking windows this drone needs. YOLO
            # still gets a chance: if it found a histogram-matching
            # detection, we prefer that and also feed it as a cheap re-init
            # to correct any KCF drift.
            if target.cls is None and target.kcf is not None:
                # KCF is the primary tracker. While it's holding the target,
                # YOLO is NOT allowed to override — otherwise a similar-looking
                # bystander's detection can steal the lock. YOLO-assist only
                # runs during a genuine loss (KCF drift / occlusion).
                kcf_box = target.tracker_update(frame)
                if kcf_box is not None:
                    target.update_found(frame, kcf_box)
                    locked_box = kcf_box
                elif best_det is not None and target.lost_frames > 0:
                    # Re-acquisition path: KCF failed, but YOLO has a
                    # histogram-matching candidate. Gate it by spatial
                    # proximity to the Kalman-predicted position so we
                    # can't accept a match on the other side of the frame.
                    pred_cx, pred_cy = target.predict_pos()
                    dx = (best_det["box"][0] + best_det["box"][2]) / 2 - pred_cx
                    dy = (best_det["box"][1] + best_det["box"][3]) / 2 - pred_cy
                    max_jump = max(target.w, target.h) * 2.0
                    if (dx * dx + dy * dy) ** 0.5 <= max_jump:
                        target.update_found(frame, best_det["box"])
                        locked_box = best_det["box"]
                        try:
                            x1, y1, x2, y2 = best_det["box"]
                            target.kcf = cv2.TrackerKCF_create()
                            target.kcf.init(frame, (int(x1), int(y1),
                                                    int(x2 - x1), int(y2 - y1)))
                            target.last_good_frame = frame.copy()
                            target.last_good_box = [int(x1), int(y1), int(x2), int(y2)]
                        except Exception:
                            pass
                    else:
                        target.update_lost()
                        is_lost = True
                        if target.lost_frames > 1500:
                            target = None
                            print("Target lost — unlocked")
                else:
                    # Hard loss: no KCF result, no YOLO re-acquire.
                    # Do NOT draw a stale predicted box — users need to
                    # see immediately that we don't know where the target
                    # is. UI shows "SEARCHING..." text only until either
                    # re-acquisition or hard-drop at 1500 frames.
                    target.update_lost()
                    if target.lost_frames <= 1500:
                        is_lost = True
                    else:
                        target = None
                        print("Target lost — unlocked")
            elif best_det is not None:
                # YOLO-class lock, YOLO found the target this frame.
                target.update_found(frame, best_det["box"])
                locked_box = best_det["box"]
            else:
                # YOLO-class lock, YOLO missed this frame — Kalman predict
                # and keep histogram-matching incoming dets for reacquisition.
                target.update_lost()
                if target.lost_frames <= 10:
                    locked_box = target.predicted_box()
                    is_lost = True
                elif target.lost_frames <= 300:
                    is_lost = True
                else:
                    target = None
                    print("Target lost — unlocked")

        # Draw YOLO detections — subtle 1px outline, small label above.
        for d in dets:
            x1, y1, x2, y2 = d["box"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (180, 180, 180), 1, cv2.LINE_AA)
            _draw_label(frame, f"{d['cls']} {int(d['conf']*100)}%",
                        x1, y1, (220, 220, 220))

        # Draw locked target as a HUD-style corner-bracket reticle.
        if target is not None and locked_box:
            label = f"LOCK · {target.cls.upper()}" if target.cls else "LOCK"
            _draw_reticle(frame, locked_box, (120, 255, 180), label)

        # Silent search — no box, small status chip top-centered.
        if target is not None and locked_box is None and is_lost:
            _draw_chip(frame, "SEARCHING", 8, 8 + 22 + 6, (80, 180, 255))

        # FPS
        fps_count += 1
        elapsed = time.time() - fps_t
        if elapsed >= 1.0:
            shown_fps = fps_count / elapsed
            fps_count = 0
            fps_t = time.time()
        _draw_chip(frame, f"{shown_fps:.0f} FPS", 8, 8, (230, 230, 230))

        # On-demand JPEG encoding: only spend the ~5 ms per frame on cv2.imencode
        # if a /frame consumer has been active in the last 2 seconds. Saves
        # ~10% of one CPU core when nobody's watching the annotated stream
        # (the typical case — dashboard tracking tile uses drone-control's
        # raw stream, not vision-detect's annotated /frame). Inference, dets
        # publishing on /state, and tracker logic all run unconditionally;
        # only the JPEG encode is gated. seq still increments every frame
        # (heartbeat for vision-lock contract rule 2b). When a consumer
        # comes back after silence, the first /frame returns 503; the next
        # one (within ~50 ms) returns the freshly encoded JPEG.
        with _state_lock:
            last_frame_req = _state.get("last_frame_request", 0)
        encode_now = (time.monotonic() - last_frame_req) < 2.0

        if encode_now:
            # Quality 55 keeps the 720p feed comparable in bytes to the old
            # 480p @ q75. The UI needs visible structure, not photographic fidelity.
            _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
            jpg_bytes = jpg.tobytes()
        else:
            jpg_bytes = None

        f_h, f_w = frame.shape[:2]
        with _state_lock:
            if jpg_bytes is not None:
                _state["jpg"] = jpg_bytes
            _state["dets"] = dets
            _state["seq"] += 1
            _state["tracking"] = target is not None
            _state["lost"] = is_lost
            _state["locked_box"] = locked_box
            _state["cls"] = target.cls if target is not None else None
            _state["frame_w"] = int(f_w)
            _state["frame_h"] = int(f_h)


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/frame":
            # Bump the consumer-presence timestamp so capture_loop knows to
            # keep re-encoding JPEGs. See B1 optimization at line ~818.
            with _state_lock:
                _state["last_frame_request"] = time.monotonic()
                jpg = _state.get("jpg")
            if jpg is None:
                self.send_error(503); return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg)
        elif self.path == "/state":
            # Read _latest_ts under its own lock; computing the age outside
            # both locks keeps the critical sections short.
            with _latest_lock:
                latest_ts = _latest_ts
            now_mono = time.monotonic()
            frame_age_ms = int((now_mono - latest_ts) * 1000) if latest_ts > 0 else -1
            with _state_lock:
                data = json.dumps({
                    "dets": _state["dets"],
                    "tracking": _state.get("tracking", False),
                    "lost": _state.get("lost", False),
                    "locked_box": _state.get("locked_box"),
                    "seq": _state.get("seq", 0),
                    # Vision-lock contract additive fields:
                    "cls": _state.get("cls"),
                    "frame_w": _state.get("frame_w", 0),
                    "frame_h": _state.get("frame_h", 0),
                    "frame_age_ms": frame_age_ms,
                })
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data.encode())
        elif self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE.encode())
        else:
            self.send_error(404)

    def do_POST(self):
        global _lock_req
        if self.path == "/lock":
            body = self.rfile.read(int(self.headers["Content-Length"]))
            data = json.loads(body)
            with _lock_req_lock:
                _lock_req = data
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_error(404)

    def log_message(self, *a):
        pass


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    request_queue_size = 16


PAGE = """<!DOCTYPE html>
<html><head><title>Chase</title>
<style>
*{margin:0;box-sizing:border-box}
body{background:#111;color:#fff;font-family:monospace;display:flex;flex-direction:column;height:100vh;user-select:none}
#top{display:flex;flex:1;overflow:hidden}
#stream-wrap{flex:1;position:relative;display:flex;justify-content:center;align-items:center;background:#000}
#stream{max-width:100%;max-height:100%;cursor:crosshair}
#sel{position:absolute;border:2px dashed #0f0;display:none;pointer-events:none}
#panel{width:260px;padding:10px;overflow-y:auto;background:#1a1a1a;border-left:1px solid #333}
.det{padding:6px 8px;margin:2px 0;border-radius:4px;cursor:pointer;font-size:13px;border:1px solid #333;display:flex;justify-content:space-between}
.det:hover{background:#333}
#status{position:absolute;top:10px;left:10px;background:#000a;padding:4px 10px;border-radius:4px;font-size:13px}
#unlock{display:none;position:absolute;bottom:10px;left:50%;transform:translateX(-50%);
  background:#e00;color:#fff;border:none;padding:8px 20px;border-radius:4px;cursor:pointer;font-size:14px;font-family:monospace}
#hint{position:absolute;bottom:10px;left:10px;background:#000a;padding:4px 10px;border-radius:4px;font-size:12px;color:#888}
</style>
</head><body>
<div id="top">
  <div id="stream-wrap">
    <img id="stream" draggable="false">
    <div id="sel"></div>
    <div id="status">Detecting...</div>
    <button id="unlock" onclick="doUnlock()">UNLOCK</button>
    <div id="hint">Drag a box or click a detection to lock</div>
  </div>
  <div id="panel"><b>Detections</b><div id="dets"></div></div>
</div>
<script>
const img=document.getElementById('stream');
const sel=document.getElementById('sel');
let locked=false,dragging=false,sx,sy;

function fetchFrame(){
  fetch('/frame').then(r=>r.blob()).then(blob=>{
    const url=URL.createObjectURL(blob);
    img.onload=()=>{URL.revokeObjectURL(url);fetchFrame()};
    img.onerror=()=>{URL.revokeObjectURL(url);setTimeout(fetchFrame,100)};
    img.src=url;
  }).catch(()=>setTimeout(fetchFrame,100));
}
fetchFrame();

function toImg(ex,ey){
  const r=img.getBoundingClientRect();
  return[(ex-r.left)*img.naturalWidth/r.width,(ey-r.top)*img.naturalHeight/r.height];
}
img.addEventListener('mousedown',e=>{
  e.preventDefault();dragging=true;sx=e.clientX;sy=e.clientY;
  sel.style.left=sx+'px';sel.style.top=sy+'px';
  sel.style.width='0';sel.style.height='0';sel.style.display='block';
});
document.addEventListener('mousemove',e=>{
  if(!dragging)return;
  sel.style.left=Math.min(sx,e.clientX)+'px';sel.style.top=Math.min(sy,e.clientY)+'px';
  sel.style.width=Math.abs(e.clientX-sx)+'px';sel.style.height=Math.abs(e.clientY-sy)+'px';
});
document.addEventListener('mouseup',e=>{
  if(!dragging)return;dragging=false;sel.style.display='none';
  const[x1,y1]=toImg(Math.min(sx,e.clientX),Math.min(sy,e.clientY));
  const[x2,y2]=toImg(Math.max(sx,e.clientX),Math.max(sy,e.clientY));
  if(x2-x1>10&&y2-y1>10)lockBox(x1,y1,x2-x1,y2-y1,null);
});
function lockBox(x,y,w,h,cls){
  fetch('/lock',{method:'POST',body:JSON.stringify({box:[x,y,w,h],cls})}).then(r=>r.json()).then(()=>{
    locked=true;document.getElementById('unlock').style.display='block';
    document.getElementById('hint').style.display='none';
  });
}
function lockDet(d){lockBox(d.box[0],d.box[1],d.box[2]-d.box[0],d.box[3]-d.box[1],d.cls)}
function doUnlock(){
  fetch('/lock',{method:'POST',body:JSON.stringify({unlock:true})}).then(()=>{
    locked=false;document.getElementById('unlock').style.display='none';
    document.getElementById('status').textContent='Detecting...';
    document.getElementById('status').style.color='#fff';
    document.getElementById('hint').style.display='block';
  });
}
function pollState(){
  fetch('/state').then(r=>r.json()).then(s=>{
    if(s.dets&&s.dets.length>0)
      document.getElementById('dets').innerHTML=s.dets.map(d=>
        `<div class="det" onclick='lockDet(${JSON.stringify(d)})'><span>${d.cls}</span><span>${(d.conf*100).toFixed(0)}%</span></div>`
      ).join('');
    if(s.tracking){
      document.getElementById('status').textContent=s.lost?'PREDICTED...':'LOCKED';
      document.getElementById('status').style.color=s.lost?'#ff0':'#0f0';
      document.getElementById('unlock').style.display='block';locked=true;
    }else if(locked){
      locked=false;document.getElementById('unlock').style.display='none';
      document.getElementById('status').textContent='Target lost';
      document.getElementById('status').style.color='#f00';
      document.getElementById('hint').style.display='block';
      setTimeout(()=>{if(!locked){document.getElementById('status').textContent='Detecting...';document.getElementById('status').style.color='#fff';}},2000);
    }
  }).catch(()=>{});
  setTimeout(pollState,250);
}
pollState();
</script>
</body></html>"""

if __name__ == "__main__":
    # Grabber runs independently of inference so video never backlogs.
    threading.Thread(target=grabber_loop, daemon=True).start()
    threading.Thread(target=capture_loop, daemon=True).start()
    print(f"http://localhost:{PORT}")
    Server((HOST, PORT), Handler).serve_forever()
