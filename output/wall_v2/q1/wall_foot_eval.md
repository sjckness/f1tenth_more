# Wall-foot feasibility — `corridor10m_20261016T144537`

367 depth frames. Camera zed2_left_camera_optical_frame at [0.2294, 0.06, 0.2196] m, yaw 0.0 deg.

Foot points per frame: median 229.0, p90 265.4.

## Column rejections (totals over the window)

| reason | columns |
|---|---|
| no_transition | 36854 |
| occluded | 0 |
| reopened | 1231 |
| self_mask | 0 |
| horizon | 0 |
| too_far | 83254 |
| inferred_floor_cap | 32196 |

## Timing

| stage | p50 [ms] | p95 [ms] |
|---|---|---|
| foot_ms | 3.44 | 3.63 |
| scan_ms | 1.86 | 2.12 |
| total_ms | 5.27 | 5.66 |

## Range sensitivity of the foot — d(range)/d(row)

One pixel of foot-row error, in metres of range. This is P1's own
sigma_r = d^2 / (h fy), with h the camera height.

| range | rows below the horizon | m per pixel |
|---|---|---|
| 1.0m | 57.6 | 0.0173 |
| 1.5m | 38.4 | 0.039 |
| 2.0m | 28.8 | 0.0694 |
| 3.0m | 19.2 | 0.1561 |
| 4.0m | 14.4 | 0.2776 |
| 5.0m | 11.5 | 0.4337 |

## Point residual [m] — nearest LiDAR return to each camera point

| source | 0.75-1.5 m | 1.5-2.5 m | 2.5-4.0 m |
|---|---|---|---|
| foot | 0.1451/0.1564 (n=21) | 0.1911/0.3346 (n=18554) | 0.2019/0.3785 (n=40569) |
| stereo | 0.0487/0.2208 (n=4594) | 0.1134/0.359 (n=56304) | 0.2142/0.3369 (n=78089) |

## Point residual in PIXELS (residual / metres-per-pixel at that range)

| source | 0.75-1.5 m | 1.5-2.5 m | 2.5-4.0 m |
|---|---|---|---|
| foot | 3.7782/4.0748 (n=21) | 2.3777/4.3877 (n=18554) | 0.9612/2.5252 (n=40569) |
| stereo | 1.5154/7.9154 (n=4594) | 1.7314/3.8417 (n=56304) | 1.2139/1.8239 (n=78089) |

## Foot range histogram

| range [m] | points |
|---|---|
| 1.25-1.50 | 21 |
| 1.50-1.75 | 1344 |
| 1.75-2.00 | 3011 |
| 2.00-2.25 | 4300 |
| 2.25-2.50 | 9899 |
| 2.50-2.75 | 11501 |
| 2.75-3.00 | 1860 |
| 3.25-3.50 | 307 |
| 3.50-3.75 | 7990 |
| 3.75-4.00 | 18911 |
| 4.00-4.25 | 13408 |
| 4.25-4.50 | 8793 |

## Per source

| source | radial | residual 0.75-1.5 m | residual 1.5-2.5 m | residual 2.5-4.0 m | angle med | >20 deg | spurious/frame | left % | right % | kept | dropped |
|---|---|---|---|---|---|---|---|---|---|---|---|
| foot_0.5s | on | -- | 0.1664/0.1934 (n=10) | 0.2537/0.4066 (n=3) | 2.9138 | 0.0 % | 0.643 | 0.8 | 2.2 | 282 | 11 |
| foot_0.5s | off | -- | 0.1664/0.1934 (n=10) | 0.2094/0.3875 (n=4) | 2.9138 | 0.0 % | 0.668 | 0.8 | 2.5 | 293 | 0 |
| foot_1.0s | on | -- | 0.1649/0.1872 (n=8) | 0.4046/0.412 (n=3) | 3.0375 | 0.0 % | 0.643 | 1.4 | 1.6 | 255 | 21 |
| foot_1.0s | off | -- | 0.1649/0.1872 (n=8) | 0.3914/0.4083 (n=7) | 3.079 | 0.0 % | 0.678 | 1.4 | 1.6 | 276 | 0 |
| stereo_old | on | -- | 0.1341/0.3519 (n=12) | 0.3229/0.3653 (n=43) | 2.4582 | 0.0 % | 0.112 | 14.2 | 5.7 | 99 | 437 |
| stereo_old | off | -- | 0.1305/0.3443 (n=16) | 0.2995/0.3652 (n=165) | 5.6971 | 0.0 % | 0.932 | 58.9 | 25.6 | 536 | 0 |
| merged | on | -- | 0.1502/0.1896 (n=18) | 0.2094/0.3875 (n=4) | 2.9214 | 0.0 % | 0.643 | 1.1 | 2.5 | 292 | 20 |
| merged | off | -- | 0.1401/0.1877 (n=20) | 0.0871/0.2919 (n=9) | 2.9331 | 0.0 % | 0.668 | 1.4 | 2.7 | 312 | 0 |
