# f1tenth_comms

The car's side of the link to the base PC. Setup, parameters, the Foxglove
view and the test order on the car: [output/comms/setup.md](../../output/comms/setup.md).

| Node | What it does | Topics |
|---|---|---|
| `wifi_coverage_node` | 1 Hz: RSSI of the Wi-Fi interface, RTT/loss/LOST from `/comms/link_status`, binned per 1 m cell of `map` (TF `map -> base_link`). Session only. | `/comms/coverage_rssi`, `/comms/coverage_rtt` (OccupancyGrid, latched), `/comms/coverage_cells` (JSON), `/comms/coverage_reset` (Trigger) |

Both grids: `-1` never visited / no data, `0..100` good..bad, `-128` no link.
The scaling is in `f1tenth_comms/coverage.py` and `config/comms.yaml`.

Started by the supervisor as the `comms` component only with
`enable_comms: true` in `stack_params.yaml` (default `false`). Nothing in
the stack reads `/comms/*` to drive, stop or abort.
