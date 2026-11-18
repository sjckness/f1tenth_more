# Wall-foot feasibility — `bag`

126 depth frames. Camera zed2_left_camera_optical_frame at [0.2299, 0.06, 0.22] m, yaw 0.0 deg.

Foot points per frame: median 245.0, p90 288.5.

## Column rejections (totals over the window)

| reason | columns |
|---|---|
| no_transition | 13944 |
| occluded | 0 |
| reopened | 307 |
| self_mask | 541 |
| horizon | 0 |
| too_far | 34961 |

## Timing

| stage | p50 [ms] | p95 [ms] |
|---|---|---|
| foot_ms | 3.15 | 3.19 |
| scan_ms | 1.84 | 2.18 |
| total_ms | 4.99 | 5.33 |

## Range sensitivity of the foot — d(range)/d(row)

One pixel of foot-row error, in metres of range. This is P1's own
sigma_r = d^2 / (h fy), with h the camera height.

| range | rows below the horizon | m per pixel |
|---|---|---|
| 1.0m | 57.7 | 0.0173 |
| 1.5m | 38.5 | 0.039 |
| 2.0m | 28.9 | 0.0693 |
| 3.0m | 19.2 | 0.1558 |
| 4.0m | 14.4 | 0.2771 |
| 5.0m | 11.5 | 0.4329 |

## Point residual — nearest LiDAR return to each camera point

| source | 0-1.5 m | 1.5-3 m | 3-5 m |
|---|---|---|---|
| foot | 0.2327/0.2882 (n=3451) | 0.2495/0.675 (n=17592) | 0.2338/0.5461 (n=9847) |
| stereo | 0.0996/0.2478 (n=11047) | 0.1468/0.5839 (n=20707) | 0.1829/1.2107 (n=13853) |

## Per source

| source | radial | residual 0-1.5 med/p90 | 1.5-3 med/p90 | 3-5 med/p90 | angle med | >20 deg | spurious/frame | left % | right % | kept | dropped |
|---|---|---|---|---|---|---|---|---|---|---|---|
| foot_0.5s | on | -- | 0.1972/0.2419 (n=4) | 0.4329/0.476 (n=2) | 8.1683 | 0.0 % | 0.04 | 0.8 | 2.4 | 11 | 15 |
| foot_0.5s | off | -- | 0.1718/0.2186 (n=8) | 0.1496/0.4437 (n=5) | 2.6478 | 0.0 % | 0.103 | 2.4 | 3.2 | 26 | 0 |
| foot_1.0s | on | -- | 0.1873/0.2175 (n=4) | 0.1985/0.1985 (n=1) | 2.6936 | 0.0 % | 0.063 | 0.8 | 0.8 | 13 | 20 |
| foot_1.0s | off | -- | 0.1641/0.202 (n=9) | 0.1188/0.4389 (n=9) | 3.9828 | 0.0 % | 0.119 | 1.6 | 1.6 | 33 | 0 |
| stereo_old | on | -- | 0.114/0.1243 (n=9) | 0.0653/0.0977 (n=7) | 7.1962 | 0.0 % | 0.119 | 8.7 | 3.2 | 31 | 98 |
| stereo_old | off | -- | 0.1161/0.2164 (n=14) | 0.0714/0.144 (n=19) | 8.2449 | 0.0 % | 0.762 | 50.8 | 17.5 | 129 | 0 |
| merged | on | -- | 0.1198/0.2004 (n=12) | 0.0764/0.4113 (n=8) | 6.8247 | 0.0 % | 0.04 | 4.8 | 2.4 | 25 | 21 |
| merged | off | -- | 0.1214/0.1947 (n=19) | 0.0755/0.3101 (n=14) | 6.2287 | 0.0 % | 0.103 | 8.7 | 3.2 | 46 | 0 |
