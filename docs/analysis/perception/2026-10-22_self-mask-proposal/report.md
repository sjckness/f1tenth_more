---
id: 2026-10-22_self-mask-proposal
title: "Self mask: com'è oggi, quanto costa, cosa metterci"
area: perception
date: 2026-10-22
tags: [self-mask, scene-mask, front-clearance, zed, hsv-trace]
modules: {f1tenth_perception: "0.0.1", f1tenth_params: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: open
hypothesis: "La self mask attuale (due poligoni frazionali) è sovradimensionata? Si può sostituire con un poligono tracciato sull'hardware reale senza perdere copertura?"
result: "Maschera attuale 14.78 % del frame (34 057 px), hardware reale 3.61 %; proposta 7.84 % (18 054 px): -6.95 punti, copre 100 % dell'hardware, 0.0 % di scena nuova nascosta. La via depth non funziona (vetro e pavimento senza match 9.96 %)."
decision: "Patch che cambia un solo default in stack_params.yaml (scene_mask_self_polygons); proposta in sola lettura, non applicata; polygon 1 (lente destra occlusa) invariato."
raw_data: "nessun dato grezzo su git. Esterni: corridor10m_20261015T171753 (155 frame depth, 28 RGB 640x360); output/self_mask_proposal/ (patch, png, trace json)"
migrated_from: "orin/orin-wip-2026-10-22:output/self_mask_proposal/self_mask_proposal.md"
migrated_on: 2026-11-18
---
# Self mask: what it is today, what it costs, and what to replace it with

**Read-only proposal. Nothing here is committed and nothing is wired in.**
The patch is `self_mask.patch` beside this file; apply it with `git apply`
only if you agree with the numbers below. Evidence:
`trace_self_mask.py` (reproduces everything), `self_mask_trace.json`,
`self_mask_standstill_t+0.0s.png`, `self_mask_standstill_t+9.9s.png`,
`self_mask_moving_t+34.5s.png`, `self_mask_footprint.png`.

Run: `corridor10m_20261015T171753`, 155 depth frames and 28 clean RGB frames
at 640x360.

---

## 1. What the self mask is today

It is **two static polygons in FRACTIONAL image coordinates**, rasterised by
`scene_mask.polygon_mask` at the DEPTH image's resolution and cached per
`(h, w)`. Fractions rather than pixels so one calibration survives a
resolution change with no rescaling. They are built in
`front_clearance_node.__init__`:

| # | source | fractional vertices | at 640x360 | % of frame |
|---|---|---|---|---|
| 0 | `front_clearance_scene_mask_self_polygons`, **empty by default** → derived from `lidar_exclusion_x_min/_y_min/_x_max/_y_max` | `(0.75, 0.55) (1.0, 0.55) (1.0, 1.0) (0.75, 1.0)` | x 480–640, y 198–360 | **11.25** |
| 1 | `front_clearance_scene_mask_occluded_polygons` | `(0.0, 0.78) (0.16, 0.78) (0.16, 1.0) (0.0, 1.0)` | x 0–102, y 281–360 | **3.53** |

They do not overlap: **14.78 % of the frame** (34 057 px).

An explicit JSON list in `scene_mask_self_polygons` **replaces** polygon 0;
`scene_mask_occluded_polygons` is always **appended**. An *empty* list in
either is falsy and does not clear anything — polygon 0 falls back to the
derived rectangle (there is a regression test for exactly that).

### Who uses it

| consumer | uses |
|---|---|
| `scene_mask.classify` | SELF is the **highest-priority class**: it wins over OBJECT, FLOOR, WALL and even over invalid depth |
| `inferred_floor.build_lut` (part A) | SELF pixels are excluded from the LUT, so no floor is ever inferred on the car's own hardware |
| `scene_mask.overlay` + the debug video legend | drawn red |
| `/perception/scene_labels/stats` → `mission_logger` | its own `self` percentage, in every bag |
| `wall_points` / `StereoColumns` | indirectly: a SELF pixel is never a WALL pixel, so it never reaches the column profile or the cloud |

Two **other** consumers read the same four `lidar_exclusion_*` fractions but
as a plain rectangle, not as these polygons, and this proposal does not touch
them: `front_clearance_node._lidar_exclusion_roi_mask` (the ROI path) and
`yolo_detector_node`'s detection suppression. Changing
`scene_mask_self_polygons` therefore changes the scene mask only. That is the
conservative scope, but it does mean the oversized rectangle survives in two
other places; see *Follow-ups*.

---

## 2. Is it oversized? Yes, grossly — and it is also in the wrong place

| measure | value |
|---|---|
| current mask | **14.78 %** of the frame |
| of that, live scene (usable depth ≥ 0.3 m in ≥ half the frames) | **74.4 %** of the mask = **11.0 %** of the frame |
| the car's actual hardware in the frame | **3.61 %** of the frame |
| hardware outside the current mask | **0.0 %** — it does cover all of it |

Polygon 0 was aimed at the LiDAR housing, which really is in the bottom-right
corner, but it is a *rectangle around a rounded cap that touches two frame
edges*: three quarters of it is floor, the foot of the fire extinguisher and
the right-hand glass wall at a median depth of 1.5 m. The before/after images
show it directly.

### The depth route does not work on this run, and that is worth knowing

The obvious way to trace the mask is "pixels with no usable depth, or depth
under ~0.3 m, in nearly every frame". On this recording it fails:

- **no pixel is -Inf in even a tenth of the frames** (377 px of 230 400 are
  -Inf in at least one frame, none persistently), and
- **not one finite depth anywhere in the run is under 0.3 m**.

The housing is 0.3–1.5 m from the lens and the stereo matches it perfectly
well. Meanwhile **9.96 % of the frame has persistently no depth at all**, and
`self_mask_footprint.png` shows what that is: the **glass** walls of this
corridor and the textureless floor. 7.74 % of the frame is persistent
no-match *outside* the current mask, which an automatic trace would have
masked for no reason.

**So depth alone cannot tell the car's own hardware from a pane of glass
here, and it must not be used to place this mask.** (It would work on a run
where something textured sits inside 0.2 m of the lens and comes back -Inf;
that is the clip to record if you want the automatic route.)

### What does work

The housing is an orange cap against a grey-and-white corridor and is the only
saturated orange in the frame. `trace_self_mask.py` segments it in HSV on the
left image, requires it to be present in **every** frame (the mount is rigid),
and takes the component's **own contour** — not its convex hull, which on a
cap touching two frame edges costs nothing but which on a diagonal strut
would swallow a triangle of live scene. (The hull version was tried first and
produced a 24 % mask; the numbers are in the git history of this file's first
draft, not kept.)

---

## 3. The proposal

**Derive it semi-automatically from a reference frame — which is what this
is — and then eyeball the overlay before trusting it.** Not by hand: the
colour trace is reproducible, it is objective about the silhouette, and the
mount being rigid means one trace is permanent. But it is one recording in
one corridor, so the margin is not optional.

One polygon, 9 vertices, **8 px of margin at full resolution** (parameter
`--margin-px`, default 8), in fractions of width and height:

```json
[[[0.8172, 0.7833], [0.8125, 0.8028], [0.8156, 0.9], [0.8359, 0.95],
  [0.8703, 0.9972], [0.9984, 0.9972], [0.9984, 0.7611], [0.8812, 0.7444],
  [0.8359, 0.7583]]]
```

**Polygon 1 (the occluded right lens) is KEPT unchanged.** It covers a
persistently unmatched region, which is what it claims to be — but on this run
that is indistinguishable from the textureless floor it also sits on, and the
before/after images show plain floor under it. It cannot be confirmed or
refuted here; confirm it on a clip in a textured room before shrinking it.

### What it costs and what it buys

| | current | proposed | change |
|---|---|---|---|
| mask, % of frame | 14.78 | **7.84** | **−6.95 points freed** |
| mask, px | 34 057 | 18 054 | −16 003 |
| covers the hardware | 100 % | **100 %** | unchanged |
| newly masked (scene lost) | — | **0.0 %** | nothing new is hidden |
| traced polygon alone | 11.25 (rectangle) | **4.30** | |

Nearly 7 % of the frame — most of it floor directly in front of the car, in
the rows the inferred floor and the column profile care about most — comes
back into the pipeline, and nothing that was visible becomes hidden.

---

## 4. Before / after

- `self_mask_standstill_t+0.0s.png` — clean `/camera/image_raw`, the one to
  judge by
- `self_mask_standstill_t+9.9s.png` — standstill, detector-annotated base
- `self_mask_moving_t+34.5s.png` — mid-moving-window, detector-annotated base
  (no clean RGB exists after t = 27.6 s in this bag)
- `self_mask_footprint.png` — red: the current mask; green: the hardware;
  dark grey: persistent no-stereo-match (the glass and the floor)

---

## 5. The patch

`self_mask.patch` changes **one parameter default** in
`src/f1tenth_params/config/stack_params.yaml`. **No code change is needed**:
`front_clearance_node._parse_polygons` already loads a JSON polygon list from
this parameter and already prefers it over the derived rectangle, and
`detection.launch.py` already forwards it. The loading half of the work was
done when the parameter was introduced.

Apply with:

```
git apply output/self_mask_proposal/self_mask.patch
```

Nothing else changes. To undo it, set the default back to `""`.

### Before applying, decide these

1. **The `lidar_exclusion_*` rectangle stays oversized for the other two
   consumers** (the ROI path and YOLO's detection suppression). Shrinking it
   there is a separate change with its own blast radius — YOLO would start
   accepting detections in a region it currently drops.
2. **One corridor, one lighting condition.** The colour trace depends on the
   cap being the only saturated orange in view. In a room with orange
   furniture it would pick that up too; the persistence requirement across
   every frame of a 75 s recording is what makes it safe here, not the colour
   rule on its own.
3. **The 8 px margin is a guess**, not a measurement: it covers the mount
   flexing, a different lens and the contour being a few pixels tight where
   the silhouette is concave. At 640x360 it is 1.3 % of the width.
4. **Part A's LUT excludes SELF**, so this change also gives the inferred
   floor ~7 % more of the frame to work with — in exactly the rows where the
   floor is. That is a behaviour change on the class fractions, not only on a
   debug colour.
