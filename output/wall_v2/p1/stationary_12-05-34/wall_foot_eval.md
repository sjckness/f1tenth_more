# Wall-foot feasibility — `bag`

335 depth frames. Camera zed2_left_camera_optical_frame at [0.2299, 0.06, 0.22] m, yaw 0.0 deg.

Foot points per frame: median 427.0, p90 432.0.

## Column rejections (totals over the window)

| reason | columns |
|---|---|
| no_transition | 8992 |
| occluded | 0 |
| reopened | 1933 |
| self_mask | 0 |
| horizon | 0 |
| too_far | 61220 |

## Timing

| stage | p50 [ms] | p95 [ms] |
|---|---|---|
| foot_ms | 2.54 | 3.28 |
| scan_ms | 2.3 | 2.54 |
| total_ms | 4.93 | 5.67 |

## Range sensitivity of the foot — d(range)/d(row)

One pixel of foot-row error, in metres of range. This is P1's own
sigma_r = d^2 / (h fy), with h the camera height.

| range | rows below the horizon | m per pixel |
|---|---|---|
| 1.0m | 57.7 | 0.0173 |
| 1.5m | 38.5 | 0.039 |
| 2.0m | 28.9 | 0.0693 |
| 3.0m | 19.2 | 0.1559 |
| 4.0m | 14.4 | 0.2771 |
| 5.0m | 11.5 | 0.433 |

## Point residual — nearest LiDAR return to each camera point

| source | 0-1.5 m | 1.5-3 m | 3-5 m |
|---|---|---|---|
| foot | 1.187/1.2124 (n=3433) | 0.3423/0.4044 (n=30793) | 0.3946/0.5418 (n=108029) |
| stereo | 1.2838/1.3055 (n=41) | 0.0669/0.2037 (n=113373) | 0.2227/0.8833 (n=62385) |

## Per source

| source | radial | residual 0-1.5 med/p90 | 1.5-3 med/p90 | 3-5 med/p90 | angle med | >20 deg | spurious/frame | left % | right % | kept | dropped |
|---|---|---|---|---|---|---|---|---|---|---|---|
| foot_0.5s | on | -- | -- | -- | None | 0.0 % | 0.415 | 26.0 | 12.2 | 139 | 0 |
| foot_0.5s | off | -- | -- | -- | None | 0.0 % | 0.415 | 26.0 | 12.2 | 139 | 0 |
| foot_1.0s | on | -- | -- | -- | None | 0.0 % | 0.672 | 18.2 | 48.1 | 225 | 0 |
| foot_1.0s | off | -- | -- | -- | None | 0.0 % | 0.672 | 18.2 | 48.1 | 225 | 0 |
| stereo_old | on | -- | 0.0304/0.1933 (n=71) | 0.1843/0.1972 (n=228) | 3.2165 | 0.0 % | 0.072 | 72.8 | 2.4 | 323 | 613 |
| stereo_old | off | -- | 0.0304/0.1933 (n=71) | 0.2337/0.2492 (n=507) | 6.3033 | 0.0 % | 1.069 | 99.1 | 2.4 | 936 | 0 |
| merged | on | -- | 0.0295/0.1865 (n=61) | 0.1833/0.1938 (n=214) | 3.1913 | 0.0 % | 0.415 | 74.0 | 12.2 | 414 | 0 |
| merged | off | -- | 0.0295/0.1865 (n=61) | 0.1833/0.1938 (n=214) | 3.1913 | 0.0 % | 0.415 | 74.0 | 12.2 | 414 | 0 |
