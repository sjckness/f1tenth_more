# f1tenth_comms

The car's side of the link to the base PC. Setup, parameters, the Foxglove
view and the test order on the car: [output/comms/setup.md](../../output/comms/setup.md).

| Node | What it does | Topics |
|---|---|---|
| `link_manager` | UDP heartbeat to the base every 0.5 s, Tailscale address first, hotspot address as fallback; OK / DEGRADED / LOST; while LOST, rate-limited `nmcli connection up <hotspot>` | `/comms/link_status` (`f1tenth_messages/LinkStatus`) |
| `wifi_coverage_node` | 1 Hz: RSSI of the Wi-Fi interface, RTT/loss/LOST from `/comms/link_status`, binned per 1 m cell of `map` (TF `map -> base_link`). Session only. | `/comms/coverage_rssi`, `/comms/coverage_rtt` (OccupancyGrid, latched), `/comms/coverage_cells` (JSON), `/comms/coverage_reset` (Trigger) |

The base runs `scripts/base/heartbeat_responder.py` (UDP echo, port 47800).

Both grids: `-1` never visited / no data, `0..100` good..bad, `-128` no link.
The scaling is in `f1tenth_comms/coverage.py` and `config/comms.yaml`.

Started by the supervisor as the `comms` component only with
`enable_comms: true` in `stack_params.yaml` (default `false`). Nothing in
the stack reads `/comms/*` to drive, stop or abort; the gateway's
"no client for 2 s -> abort" is its own.

Tests: `colcon test --packages-select f1tenth_comms`; under Humble:
`tools/test_comms_humble.sh`; the synthetic run of both nodes:
`tools/comms_synthetic_run.py`.
