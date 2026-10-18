# G5 — glass from LiDAR intensity: feasibility on `corridor10m_20261015T171753`

**Not feasible to decide on this run, and the reason is the run, not the idea.** `/scan.intensities` IS populated (G1: 1081 values on all 2292 scans, 0..4782, 86.5 % nonzero; `sensors.yaml` already sets `publish_intensity: true` against urg_node's shipped default of false). But the ground truth is one track: of 46 VERIFIED tracks only 4 are glass, 3 carry both flags over their life, and after windowing at 10 scans exactly **one** glass track (id 33, 2.74 m) yields **3 windows** against 49 from 34 normal tracks. Every AUC below is 3-against-49 and is a description, not a result.

**The headline feature cannot be measured here at all.** Max normalised intensity at |incidence| < 5° has **zero samples in either class**: over 3465 sampled beam rows the smallest incidence is 12.0° (normal) and 26.3° (glass). Nothing in this run was seen square-on. The glass is seen at 26-45° (median 37.9°).

**And at that angle the LiDAR does not see through it.** Pass-through ratio **0.000**, no-return fraction 0.000: the pane returns on essentially every beam of its span. The "glass" label came from the STEREO reporting the room behind it (E2's `beyond_share`), not from the LiDAR missing it -- so the pass-through signature this idea rests on is absent from this recording.

| feature | glass median | normal median | AUC | best single cut | balanced acc |
|---|---|---|---|---|---|
| intensity x range² (median) | 7 103 | 9 225 | 0.083 | `<= 7107` | 0.958 |
| pass-through ratio | 0.000 | 0.612 | 0.069 | `<= 0.018` | 0.948 |
| no-return fraction | 0.000 | 0.006 | 0.245 | `<= 0` | 0.755 |
| intensity-vs-incidence slope | +91 | -50 | 0.600 | `>= 70` | 0.800 |
| max int. at incidence < 5° | — | — | — | — | no samples |

**Best feature, with every caveat:** range-normalised intensity, LOW for glass (AUC 0.083 = 0.917 inverted), the physically expected sign -- less diffuse backscatter. The catch is overlap: normal walls span 2 436 to 26 565 and the one glass track sits at 7 103, so the "threshold" is a line through a single point. The slope feature points the right way (intensity rising with incidence on glass, falling on diffuse walls) but at AUC 0.600 says nothing yet. **Cost per cycle:** nil in new data -- the intensities are already in every `/scan`, and the beams of each track's span are already selected by `negative_evidence`: one `i * r²` and two means over ~200 beams per confirmed track, well under 0.1 ms a scan.

**Risks:** (1) one glass track, so none of this is validated; (2) glass is nearly always grazing here, which is the regime where it behaves like a mirror and returns well; (3) film, stickers and frames make a pane locally diffuse, and the tracker merges those into one segment; (4) intensity is uncalibrated on this unit and the 1/r² normalisation is the textbook exponent, not a fitted one; (5) a wet or polished floor edge gives the same low-intensity signature as glass.

**Proposal:** `glass_score_lidar = clip((i_ref - i_norm_median) / i_ref, 0, 1)` over a confirmed track's span, with `i_ref` ~9 000 from the normal-wall median, fused as a SECOND opinion beside E2's `beyond_share`: raise `glass` when both agree, and never on the LiDAR alone (one track of evidence cannot earn a veto over a distance). Keep it reporting-only until the run below exists.

**Record next:** the car pushed by hand down the same glazed corridor, stopping square to a pane at 1.5 / 3 / 5 m (incidence under 5° -- the case this run has zero samples of), then the same three distances at 45°, with `enable_mission_logger:=true` and a painted wall of the same ranges in the same clip as the control. Twenty seconds at each pose is 800 scans per class and turns every number above into a measurement.
