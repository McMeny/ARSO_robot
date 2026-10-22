"""Composite 1920x1080 @ 30 fps presentation frames (Section 15).

  +------------------------------------------+----------------------+
  | title                                    |   SIMULATED EEG badge|
  | main pane: chase / user_pov camera       | top-down map         |
  | with virtual MR-overlay markers          | (occupancy, footprint,|
  |                                          |  LiDAR, path, queue) |
  +------------------------------------------+----------------------+
  | BCI panel: Oz trace | rho bars | dwell   | state / mux / safety |
  +------------------------------------------+----------------------+
  | caption                                                          |
  +------------------------------------------------------------------+
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from eyesteer_core.world import rasterize

W, H = 1920, 1080
TITLE_H = 48
MAIN = (0, TITLE_H, 1248, 702)
MAP = (1248, TITLE_H, 672, 702)
BCI = (0, TITLE_H + 702, 1248, 270)
INFO = (1248, TITLE_H + 702, 672, 270)
CAP = (0, TITLE_H + 702 + 270, 1920, H - (TITLE_H + 702 + 270))

# dark-mode tokens from the validated reference palette (dataviz skill)
SURFACE = (26, 26, 25)
PANEL = (34, 34, 32)
INK = (240, 239, 236)
INK2 = (195, 194, 183)
MUTED = (120, 119, 112)
GRID = (56, 56, 53)
SERIES = [(57, 135, 229), (217, 89, 38), (25, 158, 112), (201, 133, 0), (213, 81, 129)]   # slots 1..5 (dark steps)
CANCEL_C = (195, 194, 183)
STATUS = {"good": (12, 163, 12), "warning": (250, 178, 25), "serious": (236, 131, 90), "critical": (208, 59, 59)}


def _font(size, bold=False):
    import matplotlib.font_manager as fm
    path = fm.findfont(fm.FontProperties(family="DejaVu Sans", weight="bold" if bold else "normal"))
    return ImageFont.truetype(path, size)


def marker_color(m):
    if m.cancel:
        return CANCEL_C
    try:
        return SERIES[(int(m.slot) - 1) % len(SERIES)]
    except (TypeError, ValueError):
        return CANCEL_C


class Hud:
    def __init__(self, scn: dict, out: Path, true_flicker: bool = False):
        self.scn = scn
        self.out = Path(out)
        self.true_flicker = true_flicker
        self.f = {s: _font(s) for s in (14, 16, 18, 20, 24, 28)}
        self.fb = {s: _font(s, True) for s in (14, 16, 18, 20, 24, 30, 36)}
        self.frames = 0
        self.writer = None
        self.stills_done = set()
        self.stills = []

    # ------------------------------------------------------------------ setup
    def attach(self, runner):
        self.r = runner
        sim = runner.sim
        self.renderer = mujoco.Renderer(sim.m, MAIN[3], MAIN[2])
        self.opt = mujoco.MjvOption()
        for g in range(6):
            self.opt.geomgroup[g] = 1 if g != 5 else 0
        self._build_map_base()
        (self.out / "videos").mkdir(parents=True, exist_ok=True)
        (self.out / "stills").mkdir(parents=True, exist_ok=True)
        import imageio.v2 as iio
        self.video_path = self.out / "videos" / f"{self.scn.get('id', 'S')}_{self.scn['name']}.mp4"
        self.writer = iio.get_writer(str(self.video_path), fps=30, codec="libx264", quality=7,
                                     macro_block_size=8, ffmpeg_log_level="error")

    def _build_map_base(self):
        w = self.r.sim.world
        grid = rasterize(w, 0.05, margin=0.3)
        self.grid = grid
        x0, y0 = grid.origin
        ny, nx = grid.occ.shape
        span_x, span_y = nx * grid.resolution, ny * grid.resolution
        pad = 36
        self.mscale = min((MAP[2] - 2 * pad) / span_x, (MAP[3] - 2 * pad - 30) / span_y)
        self.mx0 = MAP[0] + (MAP[2] - span_x * self.mscale) / 2
        self.my0 = MAP[1] + 30 + (MAP[3] - 30 - span_y * self.mscale) / 2
        self.mwx0, self.mwy0, self.mspan_y = x0, y0, span_y
        from scipy.ndimage import distance_transform_edt
        dist = distance_transform_edt(~grid.occ) * grid.resolution
        infl = self.r.costmap.inflation
        img = np.zeros((ny, nx, 3), np.uint8)
        img[:] = (44, 44, 42)
        img[(dist > 0) & (dist < infl)] = (58, 52, 40)
        img[grid.occ] = (120, 119, 112)
        im = Image.fromarray(img[::-1]).resize((int(span_x * self.mscale), int(span_y * self.mscale)), Image.NEAREST)
        self.map_base = im

    def w2m(self, x, y):
        return (self.mx0 + (x - self.mwx0) * self.mscale, self.my0 + (self.mspan_y - (y - self.mwy0)) * self.mscale)

    # ------------------------------------------------------------------ per frame
    def frame(self):
        snap = self.r.snapshot()
        t = snap["t"]
        self._style_markers(snap)
        cam = self.r.camera
        self.renderer.update_scene(self.r.sim.d, camera=cam, scene_option=self.opt)
        main = Image.fromarray(self.renderer.render())
        img = Image.new("RGB", (W, H), SURFACE)
        img.paste(main, (MAIN[0], MAIN[1]))
        d = ImageDraw.Draw(img, "RGBA")
        self._overlay_labels(d, snap, cam)
        self._title(d, snap)
        self._map(img, d, snap)
        self._bci(d, snap)
        self._info(d, snap)
        self._caption(d, snap)
        arr = np.asarray(img)
        self.writer.append_data(arr)
        for s in self.scn.get("stills", []):
            if s["name"] not in self.stills_done and t >= s["t"]:
                self.stills_done.add(s["name"])
                p = self.out / "stills" / f"{self.scn.get('id', 'S')}_{self.scn['name']}_{s['name']}.png"
                img.save(p)
                self.stills.append({"file": str(p.relative_to(self.out.parent)) if self.out.parent in p.parents else str(p),
                                    "t": s["t"], "caption": s["caption"]})
        self.frames += 1

    def close(self):
        self.writer.close()
        idx = self.out / "stills" / "index.json"
        allst = json.loads(idx.read_text(encoding="utf-8")) if idx.exists() else {}
        allst[self.scn["name"]] = self.stills
        idx.write_text(json.dumps(allst, indent=2), encoding="utf-8")
        return str(self.video_path), self.stills

    # ------------------------------------------------------------------ MR overlay markers
    def _style_markers(self, snap):
        r = self.r
        t = snap["t"]
        st = snap["fsm"]
        cam_id = r.sim.m.camera(r.camera).id
        cam_pos = r.sim.d.cam_xpos[cam_id]
        for m in snap["markers"]:
            c = np.array(marker_color(m)) / 255.0
            if self.true_flicker:
                n = int(round(30.0 / max(m.frequency, 1e-3)))
                on = (self.frames // max(1, n // 2)) % 2 == 0
                alpha = 0.9 if on else 0.1
            else:
                alpha = 0.45 + 0.25 * math.sin(2 * math.pi * 0.5 * t + m.id)   # slow labeled pulse (not SSVEP)
            ring = np.r_[c, 0.0]
            if m.id == snap["attended"]:
                ring = np.array([1, 1, 1, 0.55])
            if m.id in st.queue:
                ring = np.r_[c, 0.6]
            if m.id == st.goal:
                ring = np.array([0.35, 0.95, 1.0, 0.95])
            r.sim.style_marker(m.id, np.r_[0.25 + 0.75 * c, alpha], ring, face_point=cam_pos)

    def _project(self, cam, p):
        sim = self.r.sim
        cid = sim.m.camera(cam).id
        pos = sim.d.cam_xpos[cid]
        R = sim.d.cam_xmat[cid].reshape(3, 3)
        pc = R.T @ (np.asarray(p) - pos)
        depth = -pc[2]
        if depth < 0.2:
            return None
        f = (MAIN[3] / 2) / math.tan(math.radians(sim.m.cam_fovy[cid]) / 2)
        u = MAIN[2] / 2 + f * pc[0] / depth
        v = MAIN[3] / 2 - f * pc[1] / depth
        if not (0 <= u < MAIN[2] and 0 <= v < MAIN[3]):
            return None
        return MAIN[0] + u, MAIN[1] + v, depth

    def _occluded(self, cam, p) -> bool:
        """Is the marker hidden behind world geometry from this camera? (ray test, world geoms only)"""
        sim = self.r.sim
        cid = sim.m.camera(cam).id
        pos = sim.d.cam_xpos[cid].copy()
        vec = np.asarray(p, float) - pos
        dist = float(np.linalg.norm(vec))
        gid = np.zeros(1, np.int32)
        hit = mujoco.mj_ray(sim.m, sim.d, pos, vec / dist, np.array([1, 0, 0, 0, 0, 0], np.uint8), 1, -1, gid)
        return 0 <= hit < dist - 0.35

    def _overlay_labels(self, d, snap, cam):
        st = snap["fsm"]
        placed = []   # label boxes already drawn (simple de-overlap: push down)
        for m in snap["markers"]:
            pr = self._project(cam, (m.x, m.y, m.z + 0.28))
            if pr is None:
                continue
            u, v, depth = pr
            col = marker_color(m)
            hidden = self._occluded(cam, (m.x, m.y, m.z))
            txt = f"{m.label}  {m.frequency:.2f} Hz" + ("  (behind)" if hidden else "")
            fnt = self.fb[18] if depth < 6 and not hidden else self.fb[14]
            tw = d.textlength(txt, font=fnt)
            box = [u - tw / 2 - 8, v - 16, u + tw / 2 + 8, v + 12]
            for _ in range(6):
                if not any(box[0] < b[2] and b[0] < box[2] and box[1] < b[3] and b[1] < box[3] for b in placed):
                    break
                box = [box[0], box[1] + 32, box[2], box[3] + 32]
                v += 32
            placed.append(box)
            a = 110 if hidden else 255
            d.rounded_rectangle(box, 6, fill=(20, 20, 19, 140 if hidden else 200), outline=col + (a,), width=2)
            d.text((u, v - 2), txt, font=fnt, fill=INK + (a,), anchor="mm")
            if m.id == st.goal:
                d.rounded_rectangle([box[0] - 4, box[1] - 4, box[2] + 4, box[3] + 4], 8, outline=(90, 240, 255, 255), width=3)
                d.text((u, box[1] - 14), "GOAL", font=self.fb[14], fill=(90, 240, 255), anchor="mm")
            if m.id in st.queue:
                k = st.queue.index(m.id) + 1
                cx, cy = box[2] + 6, box[1]
                d.ellipse([cx - 13, cy - 13, cx + 13, cy + 13], fill=col + (255,), outline=INK, width=2)
                d.text((cx, cy), str(k), font=self.fb[16], fill=(255, 255, 255), anchor="mm")
            if m.id == snap["attended"]:
                d.text((u, box[3] + 12), "attended", font=self.f[14], fill=INK2, anchor="mm")
        lab = "MR overlay markers (virtual: slow pulse, real flicker runs only in the headset)"
        if self.true_flicker:
            lab = "TRUE FLICKER (photosensitivity warning; 30 fps aliases the real frequencies)"
        d.rounded_rectangle([MAIN[0] + 12, MAIN[1] + 12, MAIN[0] + 24 + d.textlength(lab, font=self.f[16]), MAIN[1] + 40],
                            6, fill=(20, 20, 19, 190))
        d.text((MAIN[0] + 18, MAIN[1] + 26), lab, font=self.f[16], fill=INK2, anchor="lm")
        camlab = {"chase": "chase camera", "user_pov": "user's point of view", "top": "top camera"}.get(cam, cam)
        d.text((MAIN[0] + MAIN[2] - 16, MAIN[1] + 26), camlab, font=self.f[16], fill=INK2, anchor="rm")

    # ------------------------------------------------------------------ panels
    def _title(self, d, snap):
        d.rectangle([0, 0, W, TITLE_H], fill=PANEL)
        d.text((18, TITLE_H / 2), self.scn.get("title", self.scn["name"]), font=self.fb[24], fill=INK, anchor="lm")
        d.text((760, TITLE_H / 2), "Eye as a Steering Wheel: SSVEP + Humanistic Intelligence", font=self.f[18], fill=INK2, anchor="lm")
        self._badge(d, W - 18, TITLE_H / 2, "SIMULATED EEG", anchor_right=True)

    def _badge(self, d, x, y, text, anchor_right=False, size=18):
        fnt = self.fb[size]
        tw = d.textlength(text, font=fnt)
        x0 = x - tw - 24 if anchor_right else x
        d.rounded_rectangle([x0, y - 15, x0 + tw + 24, y + 15], 8, fill=(208, 59, 59, 255))
        d.text((x0 + 12, y), text, font=fnt, fill=(255, 255, 255), anchor="lm")

    def _map(self, img, d, snap):
        d.rectangle([MAP[0], MAP[1], MAP[0] + MAP[2], MAP[1] + MAP[3]], fill=SURFACE)
        d.text((MAP[0] + 16, MAP[1] + 18), "Top-down map (map frame)", font=self.fb[18], fill=INK, anchor="lm")
        img.paste(self.map_base, (int(self.mx0), int(self.my0)))
        # tracks: ground truth (solid) vs odometry (dotted)
        gt = snap["gt_track"]
        if len(gt) > 1:
            d.line([self.w2m(*p) for p in gt[-600:]], fill=(195, 194, 183, 160), width=2)
        od = snap["odom_track"]
        for p in od[-600::3]:
            u, v = self.w2m(*p)
            d.ellipse([u - 1.5, v - 1.5, u + 1.5, v + 1.5], fill=(213, 81, 129, 200))
        # people
        for (cx, cy, rr) in self.r.sim.people_circles():
            u, v = self.w2m(cx, cy)
            rp = rr * self.mscale
            d.ellipse([u - rp, v - rp, u + rp, v + rp], fill=(217, 89, 38, 200))
        for (cx, cy, rr) in self.r.sim.extra_obstacles:
            u, v = self.w2m(cx, cy)
            rp = rr * self.mscale
            d.ellipse([u - rp, v - rp, u + rp, v + rp], fill=(160, 160, 150, 255))
        # path + goal
        if snap["path"] is not None:
            d.line([self.w2m(*p) for p in snap["path"]], fill=(90, 240, 255, 230), width=3)
        if snap["goal"] is not None:
            gx, gy = snap["goal"][2], snap["goal"][3]
            u, v = self.w2m(gx, gy)
            d.line([u - 8, v - 8, u + 8, v + 8], fill=(90, 240, 255), width=3)
            d.line([u - 8, v + 8, u + 8, v - 8], fill=(90, 240, 255), width=3)
        # LiDAR hits
        for x, y in snap["scan"][::2]:
            u, v = self.w2m(x, y)
            d.rectangle([u - 1, v - 1, u + 1, v + 1], fill=(57, 135, 229, 255))
        # robot footprint (walker: includes the user)
        x, y, yaw = snap["pose"]
        poly = [self.w2m(*p) for p in self.r.fp.world(x, y, yaw)]
        d.polygon(poly, fill=(25, 158, 112, 90), outline=(25, 158, 112, 255))
        u, v = self.w2m(x, y)
        d.line([u, v, u + 18 * math.cos(yaw), v - 18 * math.sin(yaw)], fill=INK, width=3)
        # markers + queue badges
        st = snap["fsm"]
        for m in snap["markers"]:
            u, v = self.w2m(m.x, m.y)
            col = marker_color(m)
            d.rectangle([u - 7, v - 7, u + 7, v + 7], fill=col + (255,), outline=INK if m.id == st.goal else None, width=2)
            d.text((u + 11, v - 1), m.label, font=self.fb[14], fill=INK, anchor="lm")
            if m.id in st.queue:
                k = st.queue.index(m.id) + 1
                d.ellipse([u - 22, v - 24, u - 6, v - 8], fill=col + (255,))
                d.text((u - 14, v - 16), str(k), font=self.fb[14], fill=(255, 255, 255), anchor="mm")
        # legend
        ly = MAP[1] + MAP[3] - 14
        items = [((25, 158, 112), "footprint" + (" (incl. user)" if self.r.is_walker else "")), ((90, 240, 255), "path"),
                 ((57, 135, 229), "LiDAR"), ((195, 194, 183), "ground truth"), ((213, 81, 129), "odometry")]
        x0 = MAP[0] + 16
        for col, lab in items:
            d.rectangle([x0, ly - 5, x0 + 10, ly + 5], fill=col)
            d.text((x0 + 15, ly), lab, font=self.f[14], fill=INK2, anchor="lm")
            x0 += 24 + d.textlength(lab, font=self.f[14])

    def _bci(self, d, snap):
        x0, y0, w, h = BCI
        d.rectangle([x0, y0, x0 + w, y0 + h], fill=PANEL)
        st = snap["bci"]
        # Oz trace
        tx, ty, tw, th = x0 + 16, y0 + 40, 470, h - 64
        d.text((tx, y0 + 18), "Oz (AUX cup electrode), last 4 s", font=self.fb[16], fill=INK, anchor="lm")
        oz = snap["oz"]
        oz = oz - np.mean(oz)
        scale = th / 2 / 40.0
        pts = []
        n = len(oz)
        step = max(1, n // tw)
        for i in range(0, n, step):
            pts.append((tx + i * tw / n, ty + th / 2 - float(np.clip(oz[i] * scale, -th / 2, th / 2))))
        d.rectangle([tx, ty, tx + tw, ty + th], outline=GRID)
        d.line([(tx, ty + th / 2), (tx + tw, ty + th / 2)], fill=GRID)
        d.line(pts, fill=(57, 135, 229), width=1)
        d.text((tx + 4, ty + 4), "±40 µV", font=self.f[14], fill=MUTED)
        # rho bars
        bx, by, bw, bh = x0 + 520, y0 + 44, 450, h - 96
        d.text((bx, y0 + 18), "CCA ρ per marker", font=self.fb[16], fill=INK, anchor="lm")
        markers = snap["markers"]
        nbar = len(markers)
        slot_w = bw / max(nbar, 1)
        maxr = 1.0
        for i, m in enumerate(markers):
            rho = st.rhos[i] if i < len(st.rhos) else 0.0
            col = marker_color(m)
            hb = bh * min(rho, maxr) / maxr
            bx0 = bx + i * slot_w + 8
            d.rounded_rectangle([bx0, by + bh - hb, bx0 + slot_w - 16, by + bh], 4, fill=col + (255,))
            d.text((bx0 + (slot_w - 16) / 2, by + bh + 12), m.label[:8], font=self.fb[14], fill=INK, anchor="mm")
            d.text((bx0 + (slot_w - 16) / 2, by + bh + 28), f"{m.frequency:.2f}", font=self.f[14], fill=INK2, anchor="mm")
            d.text((bx0 + (slot_w - 16) / 2, by + bh - hb - 10), f"{rho:.2f}", font=self.f[14], fill=INK2, anchor="mm")
        thr = st.threshold
        yth = by + bh - bh * thr / maxr
        for xx in range(int(bx), int(bx + bw), 10):
            d.line([(xx, yth), (xx + 5, yth)], fill=INK, width=2)
        d.text((bx + bw + 4, yth), f"θ {thr:.2f}", font=self.f[14], fill=INK, anchor="lm")
        # dwell, engagement, quality
        rx = x0 + 1030
        d.text((rx, y0 + 18), "Intent", font=self.fb[16], fill=INK, anchor="lm")
        cand = st.candidate
        lab = next((m.label for m in markers if m.id == cand), "-")
        d.text((rx, y0 + 50), f"candidate: {lab}", font=self.f[16], fill=INK2, anchor="lm")
        d.rounded_rectangle([rx, y0 + 66, rx + 200, y0 + 84], 4, outline=GRID, fill=SURFACE)
        d.rounded_rectangle([rx, y0 + 66, rx + 200 * st.dwell, y0 + 84], 4, fill=(57, 135, 229))
        d.text((rx, y0 + 98), "dwell", font=self.f[14], fill=MUTED, anchor="lm")
        d.text((rx, y0 + 126), f"engagement z: {st.engagement_z:+.2f}", font=self.f[16], fill=INK2, anchor="lm")
        d.text((rx, y0 + 150), "gate: soft", font=self.f[14], fill=MUTED, anchor="lm")
        q_ok = st.signal_ok
        col = STATUS["good"] if q_ok else STATUS["critical"]
        d.ellipse([rx, y0 + 172, rx + 14, y0 + 186], fill=col)
        d.text((rx + 20, y0 + 179), ("✓ " if q_ok else "✕ ") + st.quality[:22], font=self.f[14], fill=INK2, anchor="lm")
        state = "NO-CONTROL" if st.no_control else "attending"
        d.text((rx, y0 + 208), state, font=self.fb[16], fill=INK, anchor="lm")
        self._badge(d, rx, y0 + h - 26, "SIMULATED EEG", size=16)

    def _info(self, d, snap):
        x0, y0, w, h = INFO
        d.rectangle([x0, y0, x0 + w, y0 + h], fill=SURFACE)
        st = snap["fsm"]
        mk = {m.id: m for m in snap["markers"]}
        d.text((x0 + 16, y0 + 22), "Robot state", font=self.fb[16], fill=INK2, anchor="lm")
        disp = st.display if st.goal is None else f"ENGAGED → {mk[st.goal].label}" if st.state == "ENGAGED" else st.state
        d.text((x0 + 16, y0 + 56), disp, font=self.fb[30], fill=INK, anchor="lm")
        q = ", ".join(f"{i + 1}:{mk[g].label}" for i, g in enumerate(st.queue)) or "empty"
        d.text((x0 + 16, y0 + 94), f"queue: {q}", font=self.f[18], fill=INK2, anchor="lm")
        mux = snap["mux"]
        d.text((x0 + 16, y0 + 124), f"mux source: {mux}", font=self.f[18], fill=INK2, anchor="lm")
        cm = snap["cm"]
        col = {"OK": STATUS["good"], "SLOWDOWN": STATUS["warning"], "STOP": STATUS["critical"]}[cm]
        icon = {"OK": "✓", "SLOWDOWN": "!", "STOP": "■"}[cm]
        d.ellipse([x0 + 16, y0 + 146, x0 + 32, y0 + 162], fill=col)
        d.text((x0 + 40, y0 + 154), f"{icon} collision monitor: {cm}", font=self.f[18], fill=INK2, anchor="lm")
        tw = snap["twist"]
        d.text((x0 + 16, y0 + 186), f"v = {tw.v:+.2f} m/s   ω = {tw.w:+.2f} rad/s", font=self.f[18], fill=INK2, anchor="lm")
        if self.r.is_walker:
            g = snap["grip"]
            fl, fr = snap["force"]
            d.text((x0 + 16, y0 + 216), f"grip: {'held' if g else 'RELEASED'}   handle force: {fl:+.0f} / {fr:+.0f} N",
                   font=self.f[18], fill=INK if g else (236, 131, 90), anchor="lm")
        if snap["estop"] or st.state == "ESTOP":
            self._badge(d, x0 + w - 16, y0 + 56, "E-STOP LATCHED", anchor_right=True)
        d.text((x0 + w - 16, y0 + h - 18), f"sim time {snap['t']:6.1f} s", font=self.f[16], fill=MUTED, anchor="rm")
        if st.reason:
            import re
            reason = re.sub(r"(?<=goal |d at )(\d+)", lambda g: mk[int(g.group(1))].label if int(g.group(1)) in mk else g.group(1), st.reason)
            d.text((x0 + 16, y0 + h - 18), reason[:60], font=self.f[14], fill=MUTED, anchor="lm")

    def _caption(self, d, snap):
        x0, y0, w, h = CAP
        d.rectangle([x0, y0, x0 + w, y0 + h], fill=PANEL)
        d.text((24, y0 + h / 2), snap["caption"], font=self.f[24], fill=INK, anchor="lm")
