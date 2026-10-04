"""Combine the cart test (position_video.mp4), the real-flight replay (jury_forest_gap.mp4) and the
GNSS-denied simulation (demo.mp4) into one ~59 s pitch cut:
01 lab test -> pull-up -> 02 real flight data -> whip-pan -> 03 simulation.

Run:  python make_pitch_cut.py                 # renders TaipeiDrift_pitch_cut.mp4 (+ _720p)
      python make_pitch_cut.py --stills 5 20   # writes JPG stills at those output times into .stills/

Needs: numpy, opencv-python-headless, pillow, imageio-ffmpeg, fonttools, brotli.
"""
import argparse, math, os, subprocess, sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CART = os.path.join(HERE, "position_video.mp4")
REAL = os.path.join(HERE, "jury_forest_gap.mp4")
DEMO = os.path.join(HERE, "demo.mp4")
TRAJ = os.path.join(ROOT, "TestVideo/output/trajectory_tape_scaled.csv")
FONT_DIR = os.path.join(ROOT, "docs/pitch-offline/assets/fonts")
ARIAL_BOLD = "/System/Library/Fonts/Supplemental/Arial Bold.ttf"

W, H, FPS = 1920, 1080, 30
BG = (10, 11, 13)
INK = (244, 243, 239)
MUTED = (155, 160, 168)
DIM = (90, 95, 104)
AMBER = (255, 176, 0)
GREEN = (74, 222, 128)
RED = (255, 77, 77)
BLUE = (90, 170, 255)

# ---------------------------------------------------------------- timeline (output seconds)
T_CART = 11.0                      # B: cart test, sped up
T_TRANS1 = 2.8                     # D: 62 cm -> 100 m pull-up into the real drone photo
REAL_FPS = 4                       # R: jury_forest_gap.mp4 played whole at its own speed
REAL_N = 60
T_REAL = REAL_N / REAL_FPS
T_TRANS2 = 2.4                     # G: whip-pan into the simulation
# E: demo segments (src_start, src_end, our speed)
# demo.mp4 version of 4 Oct 10:13 (60.6 s). GNSS is lost at frame 299.
GNSS_LOSS_SRC = 299 / 30
DEMO_SEGS = [(4.0, GNSS_LOSS_SRC, 3.0),                  # take-off tail
             (GNSS_LOSS_SRC, GNSS_LOSS_SRC + 1, 1.0),    # GNSS loss hit
             (GNSS_LOSS_SRC + 1, 18.8, 2.0),             # over land, camera + range finder
             (18.8, 26.8, 2.5),    # open water, AIS window opens
             (26.8, 34.5, 2.2),    # bearings to ships
             (34.5, 46.0, 2.8),    # fused estimate
             (46.0, 51.2, 2.2),    # land again
             (51.2, 57.2, 1.5)]    # result over the helipad
REAL_H_M = 100                     # about 100 m above take-off (docs/research/tuniu-level2-results.md)
AIS_FIX_SRC = 562 / 30             # demo frame where the AIS window with the bearings to three ships appears
CAPTION_CUT = (804, 1035)          # demo frames showing the "Bearings to three ships ..." caption, blanked

B0 = 0.0
D0 = B0 + T_CART
R0 = D0 + T_TRANS1
G0 = R0 + T_REAL
E0 = G0 + T_TRANS2
_seg_t = [E0]
for s0, s1, sp in DEMO_SEGS:
    _seg_t.append(_seg_t[-1] + (s1 - s0) / sp)
TOTAL = _seg_t[-1]
HIT_T = _seg_t[1]                  # output time of the GNSS loss


def out_time(src):
    """Demo source time -> output time."""
    for (s0, s1, sp), a in zip(DEMO_SEGS, _seg_t):
        if s0 <= src < s1:
            return a + (src - s0) / sp
    raise ValueError(src)


SIG_T = out_time(AIS_FIX_SRC)      # output time of the AIS signal

CART_DUR = 47.53
CART_SPEED = CART_DUR / T_CART
CART_H_CM = 62                     # effective lens height from the tape-measure check (TestVideo README)


# ---------------------------------------------------------------- helpers
def clamp01(x): return max(0.0, min(1.0, x))
def lerp(a, b, t): return a + (b - a) * t
def seg(t, a, b): return clamp01((t - a) / (b - a))
def ease_io(x): x = clamp01(x); return 4 * x**3 if x < .5 else 1 - (-2 * x + 2) ** 3 / 2
def ease_out(x): x = clamp01(x); return 1 - (1 - x) ** 3
def ease_in(x): x = clamp01(x); return x ** 3
def ease_out_expo(x): x = clamp01(x); return 1 if x >= 1 else 1 - 2 ** (-10 * x)
def ease_in_expo(x): x = clamp01(x); return 0 if x <= 0 else 2 ** (10 * x - 10)
def lerp_rect(r0, r1, t): return tuple(lerp(a, b, t) for a, b in zip(r0, r1))


def scale_rect(r, k, cx=W / 2, cy=H / 2):
    return (cx + (r[0] - cx) * k, cy + (r[1] - cy) * k, cx + (r[2] - cx) * k, cy + (r[3] - cy) * k)


_fonts = {}
def font(size, weight=700, mono=False):
    key = (size, weight, mono)
    if key not in _fonts:
        if mono == "arial":
            f = ImageFont.truetype(ARIAL_BOLD, size)
        elif mono:
            f = ImageFont.truetype(os.path.join(FONT_DIR, "IBMPlexMono-500.ttf" if weight >= 500 else "IBMPlexMono-400.ttf"), size)
        else:
            f = ImageFont.truetype(os.path.join(FONT_DIR, "HankenGrotesk-300-700.ttf"), size)
            f.set_variation_by_axes([weight])
        _fonts[key] = f
    return _fonts[key]


def load_fonts():
    """The deck ships its fonts as woff2; PIL needs TTF, so convert once next to them in a cache dir."""
    global FONT_DIR
    cache = os.path.join(HERE, ".font_cache")
    os.makedirs(cache, exist_ok=True)
    from fontTools.ttLib import TTFont
    for n in ["HankenGrotesk-300-700", "IBMPlexMono-400", "IBMPlexMono-500"]:
        out = os.path.join(cache, n + ".ttf")
        if not os.path.exists(out):
            f = TTFont(os.path.join(FONT_DIR, n + ".woff2")); f.flavor = None; f.save(out)
    FONT_DIR = cache


_masks = {}
def round_mask(h, w, r):
    key = (h, w, r)
    if key not in _masks:
        m = np.ones((h, w), np.float32)
        if r > 0:
            m = np.zeros((h, w), np.uint8)
            cv2.rectangle(m, (r, 0), (w - 1 - r, h - 1), 255, -1)
            cv2.rectangle(m, (0, r), (w - 1, h - 1 - r), 255, -1)
            for cx, cy in [(r, r), (w - 1 - r, r), (r, h - 1 - r), (w - 1 - r, h - 1 - r)]:
                cv2.circle(m, (cx, cy), r, 255, -1, cv2.LINE_AA)
            m = m.astype(np.float32) / 255
        _masks[key] = m
    return _masks[key]


def place(dst, src, rect, alpha=1.0, radius=0, src_alpha=None):
    """Draw src into dst (float32 RGB) mapped onto a float rect, sub-pixel smooth."""
    if alpha <= 0:
        return
    x0, y0, x1, y1 = rect
    sh, sw = src.shape[:2]
    sx, sy = (x1 - x0) / sw, (y1 - y0) / sh
    if sx <= 0 or sy <= 0:
        return
    mask = round_mask(sh, sw, radius) if src_alpha is None else src_alpha
    if sx < 0.6:  # prefilter big downscales
        nw, nh = max(2, int(sw * sx * 1.5)), max(2, int(sh * sy * 1.5))
        src = cv2.resize(src, (nw, nh), interpolation=cv2.INTER_AREA)
        mask = cv2.resize(mask, (nw, nh), interpolation=cv2.INTER_AREA)
        sh, sw = nh, nw
        sx, sy = (x1 - x0) / sw, (y1 - y0) / sh
    bx0, by0 = max(0, int(math.floor(x0))), max(0, int(math.floor(y0)))
    bx1, by1 = min(W, int(math.ceil(x1))), min(H, int(math.ceil(y1)))
    if bx1 <= bx0 or by1 <= by0:
        return
    M = np.float32([[sx, 0, x0 - bx0 + 0.5 * sx - 0.5], [0, sy, y0 - by0 + 0.5 * sy - 0.5]])
    size = (bx1 - bx0, by1 - by0)
    img = cv2.warpAffine(src, M, size, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    m = cv2.warpAffine(mask, M, size, flags=cv2.INTER_LINEAR, borderValue=0)[..., None] * alpha
    reg = dst[by0:by1, bx0:bx1]
    reg[:] = reg * (1 - m) + img.astype(np.float32) * m


def zoom_blur(img, strength, cx=W / 2, cy=H / 2, n=8):
    """Radial zoom blur: average of copies scaled up to (1+strength) about (cx, cy)."""
    if strength < 0.004:
        return img
    acc = np.zeros_like(img, dtype=np.float32)
    for i in range(n):
        k = 1 + strength * i / (n - 1)
        M = np.float32([[k, 0, cx - k * cx], [0, k, cy - k * cy]])
        acc += cv2.warpAffine(img, M, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    return acc / n


def fill(dst, color, alpha=1.0, rect=None):
    x0, y0, x1, y1 = rect if rect else (0, 0, W, H)
    reg = dst[int(y0):int(y1), int(x0):int(x1)]
    reg[:] = reg * (1 - alpha) + np.array(color, np.float32) * alpha


# ---------------------------------------------------------------- text layer
class Layer:
    def __init__(self):
        self.img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        self.d = ImageDraw.Draw(self.img)

    def text(self, xy, s, f, color, alpha=1.0, anchor="la", tracking=0, shadow=False):
        if alpha <= 0:
            return
        a = int(255 * clamp01(alpha))
        if tracking:
            # manual letter spacing for small caps labels
            widths = [self.d.textlength(c, font=f) + tracking for c in s]
            total = sum(widths) - tracking
            x, y = xy
            if anchor[0] == "m": x -= total / 2
            elif anchor[0] == "r": x -= total
            for c, w_ in zip(s, widths):
                self.d.text((x, y), c, font=f, fill=color + (a,), anchor="l" + anchor[1])
                x += w_
            return
        if shadow:
            sh = Image.new("RGBA", (W, H), (0, 0, 0, 0))
            ImageDraw.Draw(sh).text((xy[0], xy[1] + 4), s, font=f, fill=(0, 0, 0, int(a * 0.6)), anchor=anchor)
            from PIL import ImageFilter
            self.img.alpha_composite(sh.filter(ImageFilter.GaussianBlur(10)))
        self.d.text(xy, s, font=f, fill=color + (a,), anchor=anchor)

    def rect(self, r, color, alpha=1.0, radius=0, outline=None, width=1):
        a = int(255 * clamp01(alpha))
        if outline:
            self.d.rounded_rectangle(r, radius=radius, outline=outline + (a,), width=width)
        else:
            self.d.rounded_rectangle(r, radius=radius, fill=color + (a,))

    def line(self, pts, color, alpha=1.0, width=2):
        self.d.line(pts, fill=color + (int(255 * clamp01(alpha)),), width=width, joint="curve")

    def comp(self, frame):
        arr = np.asarray(self.img, dtype=np.float32)
        a = arr[..., 3:4] / 255
        return frame * (1 - a) + arr[..., :3] * a


def check(L, x, y, s, color, alpha):
    L.line([(x, y + s * 0.5), (x + s * 0.38, y + s * 0.85), (x + s, y)], color, alpha, width=max(2, int(s / 6)))


# ---------------------------------------------------------------- video readers
class Reader:
    def __init__(self, path):
        self.path = path
        self.cap = cv2.VideoCapture(path)
        self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.idx = -1
        self.ret_idx = -2
        self.frame = None

    def get(self, i):
        i = int(max(0, min(self.n - 1, i)))
        if i < self.idx:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            self.idx = i - 1
        while self.idx < i:
            if not self.cap.grab():
                break
            self.idx += 1
        if self.ret_idx != self.idx:
            ok, f = self.cap.retrieve()
            if ok:
                self.frame = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
            self.ret_idx = self.idx
        return self.frame


# ---------------------------------------------------------------- cart scene
traj = np.genfromtxt(TRAJ, delimiter=",", names=True)
_k = np.ones(15) / 15
_xs = np.convolve(np.pad(traj["x_m"], 7, mode="edge"), _k, "valid")
_ys = np.convolve(np.pad(traj["y_m"], 7, mode="edge"), _k, "valid")
_dist = np.r_[0, np.cumsum(np.hypot(np.diff(_xs), np.diff(_ys)))]


def cart_state(src_t):
    i = np.interp(src_t, traj["time_s"], np.arange(len(traj)))
    j = int(round(i))
    return np.interp(i, np.arange(len(traj)), _dist), _xs[j], _ys[j]


FEED_RECT = (80, 150, 1200, 780)          # 1120 x 630
MAP_CROP = (1110, 35, 1775, 1060)         # x0, y0, x1, y1 in the source frame
_mh = 860
_mw = _mh * (MAP_CROP[2] - MAP_CROP[0]) / (MAP_CROP[3] - MAP_CROP[1])
MAP_RECT = (1575 - _mw / 2, 150, 1575 + _mw / 2, 150 + _mh)


def key_map(src):
    """Cut the floor mosaic out of its white background (outside + the hole in the middle)."""
    x0, y0, x1, y1 = MAP_CROP
    crop = src[y0:y1, x0:x1]
    white = (crop.min(axis=2) > 236).astype(np.uint8)
    n, lab = cv2.connectedComponents(white, connectivity=4)
    keep = np.zeros(n, bool)
    border = np.r_[lab[0], lab[-1], lab[:, 0], lab[:, -1]]
    keep[np.unique(border)] = True
    keep[lab[520 - y0, 1450 - x0]] = True  # the hole inside the loop, source pixel (1450, 520)
    keep[0] = False
    bgmask = keep[lab].astype(np.float32)
    bgmask = cv2.GaussianBlur(bgmask, (5, 5), 0)
    return crop, 1 - bgmask


def render_cart(cart, src_t, feed_rect=FEED_RECT, others_alpha=1.0, enter=1.0):
    fr = cart.get(src_t * 30)
    out = np.empty((H, W, 3), np.float32); out[:] = BG
    e = ease_out(enter)
    rise = (1 - e) * 40
    # map
    crop, m = key_map(fr)
    mr = MAP_RECT
    place(out, crop, (mr[0], mr[1] + rise, mr[2], mr[3] + rise), alpha=e * others_alpha, src_alpha=m)
    # feed
    feed = fr[0:540, 0:960]
    fr_ = feed_rect if feed_rect != FEED_RECT else (FEED_RECT[0], FEED_RECT[1] + rise, FEED_RECT[2], FEED_RECT[3] + rise)
    full = (fr_[2] - fr_[0]) / W
    radius = int(lerp(14, 0, clamp01((full - 0.583) / (1 - 0.583))) * 960 / max(1, fr_[2] - fr_[0]))
    place(out, feed, fr_, alpha=e, radius=max(0, radius))
    L = Layer()
    a = e * others_alpha
    dist, x, y = cart_state(src_t)
    fx0, fy0 = FEED_RECT[0], FEED_RECT[1] + rise
    # labels on the feed
    L.rect((fx0 + 18, fy0 + 18, fx0 + 18 + 390, fy0 + 58), BG, a * 0.8, radius=6)
    L.d.ellipse((fx0 + 34, fy0 + 32, fx0 + 46, fy0 + 44), fill=RED + (int(255 * a * (0.55 + 0.45 * math.cos(src_t * 4))),))
    L.text((fx0 + 58, fy0 + 38), f"PHONE CAMERA · {CART_H_CM} CM ABOVE FLOOR", font(20, 500, True), INK, a, "lm")
    L.rect((FEED_RECT[2] - 140, fy0 + 18, FEED_RECT[2] - 18, fy0 + 58), BG, a * 0.8, radius=6)
    L.text((FEED_RECT[2] - 79, fy0 + 38), f"{CART_SPEED:.1f}× SPEED", font(20, 500, True), MUTED, a, "mm")
    # HUD under the feed
    hy = FEED_RECT[3] + rise + 52
    L.text((fx0, hy), "DISTANCE MEASURED BY THE CAMERA", font(22, 500, True), MUTED, a, "la", tracking=2)
    L.text((fx0 - 6, hy + 30), f"{dist:5.2f} m", font(150, 700), INK, a, "la")
    L.text((fx0 + 640, hy + 4), "POSITION", font(22, 500, True), MUTED, a, "la", tracking=2)
    L.text((fx0 + 640, hy + 50), f"x {x*100:+7.1f} cm", font(40, 500, True), INK, a, "la")
    L.text((fx0 + 640, hy + 104), f"y {y*100:+7.1f} cm", font(40, 500, True), INK, a, "la")
    L.text((fx0 + 640, hy + 168), "60 fixes / s · no GNSS · no map", font(24, 400, True), AMBER, a, "la")
    # map caption
    L.text(((MAP_RECT[0] + MAP_RECT[2]) / 2, MAP_RECT[3] + rise + 34), "PATH, COMPUTED FROM THE CAMERA ALONE", font(20, 500, True), MUTED, a, "ma", tracking=2)
    return out, L


# ---------------------------------------------------------------- demo scene
def orig_speed(src_t):
    """Speed label burned into demo.mp4 at this source time."""
    for t1, k in [(3.4, 4), (7.8, 2), (12.7, 1), (18.8, 2), (26.8, 1), (34.5, 1.5), (46.0, 1), (51.1, 1.5)]:
        if src_t < t1:
            return k
    return 4


def demo_frame(demo, src_t, ours):
    idx = int(max(0, min(demo.n - 1, src_t * 30)))
    fr = demo.get(idx).astype(np.float32)
    fr[0:72, 0:1199] = BG          # hide the Gazebo window chrome; the chapter bar goes here
    if CAPTION_CUT[0] <= idx < CAPTION_CUT[1]:
        fr[956:1080, 0:1199] = (10, 19, 26)   # empty caption bar
    k = orig_speed(src_t) * ours
    label = "SIMULATION, real time" if abs(k - 1) < 1e-6 else f"SIMULATION, played at {k:g}x"
    img = Image.fromarray(fr[80:124, 860:1192].astype(np.uint8))
    d = ImageDraw.Draw(img)
    f = font(21, mono="arial")
    tw = d.textlength(label, font=f)
    x0 = min(36, 332 - 16 - tw - 14)
    d.rectangle((x0, 6, 332, 38), fill=(16, 18, 22))
    d.text((332 - 12, 22), label, font=f, fill=(255, 255, 255), anchor="rm")
    fr[80:124, 860:1192] = np.asarray(img, np.float32)
    return fr


def demo_src_at(t):
    """Output time in section E -> (src time, our speed)."""
    for (s0, s1, sp), a, b in zip(DEMO_SEGS, _seg_t[:-1], _seg_t[1:]):
        if t < b:
            return s0 + (t - a) * sp, sp
    s0, s1, sp = DEMO_SEGS[-1]
    return s1, sp


VIEW_FULL = (0.0, -170 * 1.6, 1920 * 1.6, -170 * 1.6 + 1080 * 1.6)  # demo frame placed so its 3D view fills the screen


# ---------------------------------------------------------------- real-flight scene
REAL_RECT = (64, 72, 1856, 1080)          # the whole original frame, below the chapter bar
_rk = (REAL_RECT[2] - REAL_RECT[0]) / W
PHOTO_C = (REAL_RECT[0] + 292 * _rk, REAL_RECT[1] + 318 * _rk)   # centre of "1. What the drone sees"


def render_real(ctx, f):
    out = np.empty((H, W, 3), np.float32); out[:] = BG
    place(out, ctx.real.get(f), REAL_RECT)
    return out


# ---------------------------------------------------------------- chapter bar
STEPS = ["LAB TEST", "REAL FLIGHT DATA", "SIMULATION"]


def chapters(L, t, band_w):
    a = 1.0
    L.rect((0, 0, band_w, 72), BG, a)
    starts = [B0, D0 + 1.7, G0 + 0.45, TOTAL + 1]   # when each step becomes active
    active = sum(t >= s for s in starts[:3]) - 1
    all_done = False
    f = font(25, 500, True)
    x, y = 40, 37
    for i, name in enumerate(STEPS):
        done = all_done or i < active
        cur = i == active and not all_done
        L.text((x, y), f"0{i + 1}", f, AMBER if cur else MUTED if done else DIM, a, "lm")
        x += 50
        L.text((x, y), name, f, INK if cur else MUTED if done else DIM, a, "lm", tracking=2)
        x += L.d.textlength(name, font=f) + 2 * len(name) + 16
        if done:
            check(L, x, y - 9, 18, AMBER, a); x += 34
        if i < len(STEPS) - 1:
            prog = seg(t, starts[i], starts[i + 1])
            L.line([(x, y), (x + 90, y)], DIM, a, 2)
            L.line([(x, y), (x + 90 * prog, y)], AMBER, a, 2)
            x += 116


# ---------------------------------------------------------------- frame composer
class Ctx:
    def __init__(self):
        self.cart = Reader(CART)
        self.demo = Reader(DEMO)
        self.real = Reader(REAL)


def popin(L, h, text, fill_col, cx=600, cy=470, dur=1.6, size=120):
    """Stamp that pops in at h = 0 and fades out at h = dur."""
    sa = ease_out(seg(h, 0.0, 0.12)) * (1 - ease_in(seg(h, dur - 0.35, dur)))
    pop = 1 + 0.35 * (1 - ease_out(seg(h, 0, 0.25)))
    f = font(int(size * pop), 700)
    tw = L.d.textlength(text, font=f)
    L.rect((cx - tw / 2 - 40 * pop, cy - 90 * pop, cx + tw / 2 + 40 * pop, cy + 90 * pop), fill_col, sa * 0.92, radius=10)
    L.text((cx, cy + 4), text, f, (255, 255, 255), sa, "mm")


def frame_at(ctx, t):
    band_w = W
    # ---------------- B: cart test
    if t < D0:
        out, L = render_cart(ctx.cart, min(CART_DUR - 0.05, (t - B0) * CART_SPEED))
        chapters(L, t, band_w)
        return L.comp(out)

    # ---------------- D: pull-up from the floor into the real drone photo
    if t < R0:
        d = t - D0
        if d < 0.9:
            # feed grows to full screen
            g = ease_io(seg(d, 0, 0.55))
            push = 1 + 0.04 * ease_io(seg(d, 0.55, 0.9))
            rect = scale_rect(lerp_rect(FEED_RECT, (0, 0, W, H), g), push)
            out, L = render_cart(ctx.cart, CART_DUR - 0.05, feed_rect=rect, others_alpha=1 - ease_out(seg(d, 0, 0.3)))
            out = L.comp(out)
            L = Layer()
            a = ease_out(seg(d, 0.45, 0.75))
            L.text((W / 2, 250), "CAMERA HEIGHT", font(30, 500, True), INK, a, "mm", tracking=8, shadow=True)
            L.text((W / 2, 375), f"{CART_H_CM} cm", font(220, 700), INK, a, "mm", shadow=True)
            fill(out, (0, 0, 0), 0.3 * a)
            chapters(L, t, band_w)
            return L.comp(out)
        # pull-up: the floor shrinks away, the real frame zooms out from its drone photo
        p = seg(d, 0.9, 1.75)
        z_hold = W / (556 * _rk)                              # drone photo fills the screen width
        z = lerp(7.0, z_hold, ease_out_expo(seg(d, 0.9, 2.0)))
        z = lerp(z, 1.0, ease_io(seg(d, 2.0, T_TRANS1)))
        k = clamp01((z - 1) / (z_hold - 1))                   # 1: photo centred, 0: frame in place
        cx, cy = lerp(PHOTO_C[0], W / 2, k), lerp(PHOTO_C[1], H / 2, k)
        lay = render_real(ctx, 0)
        out = np.empty((H, W, 3), np.float32); out[:] = BG
        place(out, lay, (cx - z * PHOTO_C[0], cy - z * PHOTO_C[1], cx + z * (W - PHOTO_C[0]), cy + z * (H - PHOTO_C[1])))
        out = zoom_blur(out, 0.25 * (1 - ease_out(seg(d, 0.9, 2.0))))
        s_ = 1.04 * (1 - ease_in_expo(p))
        if s_ > 0.002:
            fr = ctx.cart.get((CART_DUR - 0.05) * 30)[0:540, 0:960]
            layer = np.empty((H, W, 3), np.float32); layer[:] = BG
            place(layer, fr, scale_rect((0, 0, W, H), s_))
            layer = zoom_blur(layer, 0.35 * ease_in(p))
            fade = 1 - ease_in(seg(p, 0.55, 1.0))
            out = out * (1 - fade) + layer * fade
        fl = math.exp(-((d - 1.62) / 0.09) ** 2)
        out = out * (1 - 0.55 * fl) + 255 * 0.55 * fl
        L = Layer()
        # counter 62 cm -> 100 m (geometric)
        q = ease_io(seg(d, 0.95, 1.65))
        h0 = CART_H_CM / 100
        h_m = h0 * ((REAL_H_M / h0) ** q)
        txt = f"{h_m*100:.0f} cm" if h_m < 1 else f"{h_m:.0f} m"
        ca = 1 - ease_in(seg(d, 1.85, 2.15))
        fill(out, (0, 0, 0), 0.3 * ca)
        L.text((W / 2, 250), "CAMERA HEIGHT", font(30, 500, True), INK, ca, "mm", tracking=8, shadow=True)
        L.text((W / 2, 375), txt, font(220, 700), INK, ca, "mm", shadow=True)
        L.text((W / 2, 520), f"×{REAL_H_M / h0:.0f} higher", font(64, 700), AMBER, ca * ease_out(seg(d, 1.55, 1.7)), "mm", shadow=True)
        chapters(L, t, band_w)
        return L.comp(out)

    # ---------------- R: real-flight replay, the original frames
    if t < G0:
        out = render_real(ctx, min(REAL_N - 1, int((t - R0) * REAL_FPS)))
        L = Layer()
        chapters(L, t, band_w)
        return L.comp(out)

    # ---------------- G: whip-pan into the simulation
    if t < E0:
        d = t - G0
        w = ease_io(seg(d, 0, 0.45))
        dsrc = seg(d, 0, T_TRANS2) * DEMO_SEGS[0][0]
        unfold = ease_io(seg(d, 0.9, T_TRANS2))
        out = np.empty((H, W, 3), np.float32); out[:] = BG
        if w < 1:
            place(out, render_real(ctx, REAL_N - 1), (-W * w, 0, W - W * w, H))
        push = lerp(1.08, 1.0, ease_out(seg(d, 0.2, 0.9)))
        drect = scale_rect(lerp_rect(VIEW_FULL, (0, 0, W, H), unfold), push)
        off = W * (1 - w)
        place(out, demo_frame(ctx.demo, dsrc, DEMO_SEGS[0][2]), (drect[0] + off, drect[1], drect[2] + off, drect[3]))
        # horizontal motion blur from the pan speed
        kb = int(220 * math.sin(math.pi * clamp01(d / 0.45)))
        if kb > 2:
            out = cv2.blur(out, (kb, 1))
        L = Layer()
        chapters(L, t, lerp(W, 1199, unfold))
        return L.comp(out)

    # ---------------- E: simulation
    src_t, sp = demo_src_at(min(t, TOTAL - 1e-6))
    out = demo_frame(ctx.demo, src_t, sp)
    # punch-in on the result legend at the end
    pz = ease_io(seg(t, TOTAL - 1.9, TOTAL - 0.2))
    if pz > 0:
        out2 = np.empty_like(out); out2[:] = BG
        place(out2, out, scale_rect((0, 0, W, H), 1 + 0.9 * pz, 980, 800))
        out = out2
    L = Layer()
    # GNSS loss hit
    h = t - HIT_T
    if -0.02 <= h < 1.6:
        gl = math.exp(-max(h, 0) / 0.18)
        if gl > 0.03:
            sh = int(28 * gl)
            g = out.copy()
            g[..., 0] = np.roll(out[..., 0], sh, axis=1)
            g[..., 2] = np.roll(out[..., 2], -sh, axis=1)
            rng = np.random.default_rng(int(h * 30) + 7)
            for _ in range(int(10 * gl)):
                y0 = rng.integers(0, H - 40); hh = rng.integers(8, 50)
                g[y0:y0 + hh] = np.roll(g[y0:y0 + hh], int(rng.integers(-80, 80) * gl), axis=1)
            out = g * (1 - 0.45 * gl) + np.array(RED, np.float32) * 0.45 * gl
        popin(L, h, "GNSS JAMMED", (200, 30, 36))
    # AIS signal detected
    h = t - SIG_T
    if -0.02 <= h < 1.6:
        gl = math.exp(-max(h, 0) / 0.18)
        out = out * (1 - 0.35 * gl) + np.array(GREEN, np.float32) * 0.35 * gl
        popin(L, h, "SIGNAL DETECTED", (22, 140, 70), size=88)
    chapters(L, t, 1199)
    return L.comp(out)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stills", nargs="*", type=float)
    ap.add_argument("--out", default=os.path.join(HERE, "TaipeiDrift_pitch_cut.mp4"))
    args = ap.parse_args()
    load_fonts()
    ctx = Ctx()
    print(f"timeline: cart {B0:.1f}  pull-up {D0:.1f}  real {R0:.1f}  whip {G0:.1f}  sim {E0:.1f}  "
          f"GNSS jammed {HIT_T:.1f}  signal detected {SIG_T:.1f}  end {TOTAL:.1f} s")
    if args.stills is not None:
        odir = os.path.join(HERE, ".stills"); os.makedirs(odir, exist_ok=True)
        for t in args.stills:
            f = frame_at(ctx, t)
            cv2.imwrite(os.path.join(odir, f"still_{t:05.2f}.jpg"), cv2.cvtColor(np.clip(f, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        return
    import imageio_ffmpeg
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    n = int(round(TOTAL * FPS))
    cmd = [ff, "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
           "-c:v", "libx264", "-preset", "slow", "-crf", "16", "-pix_fmt", "yuv420p", "-movflags", "+faststart", args.out]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for i in range(n):
        f = frame_at(ctx, i / FPS)
        p.stdin.write(np.clip(f + 0.5, 0, 255).astype(np.uint8).tobytes())
        if i % 60 == 0:
            print(f"  {i}/{n}", flush=True)
    p.stdin.close(); p.wait()
    small = args.out.replace(".mp4", "_720p.mp4")
    subprocess.run([ff, "-v", "error", "-y", "-i", args.out, "-vf", "scale=1280:720:flags=lanczos", "-c:v", "libx264",
                    "-preset", "slow", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", small], check=True)
    print("wrote", args.out, "and", small)


if __name__ == "__main__":
    main()
