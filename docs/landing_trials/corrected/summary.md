# Landing trials

- Attempts: 30, landed and measured: 30, failed: 0
- Touchdown error from the pad centre: median 1.5 cm, mean 1.3 cm, 95th percentile 2.1 cm, worst 2.4 cm
- Inside the 1.0 m marker: 30/30; on the 1.25 m pad: 30/30
- Average miss direction: -0.3 cm east, -0.0 cm north (a consistent offset here points to a systematic cause)
- Time from start point to touchdown: mean 17.4 s, longest 21.5 s (simulation time)
- Detector range / true height of the drone above the pad at the start point: mean 1.001 (range 0.996 to 1.004, 30 readings). The camera hangs roughly 0.1 m below the point Gazebo reports, so a perfect detector would read slightly under 1.

| Batch | Landings | Median | Mean | 95th pct | Worst |
|---|---|---|---|---|---|
| batch1 | 10 | 1.5 cm | 1.5 cm | 2.1 cm | 2.1 cm |
| batch2 | 10 | 1.0 cm | 1.0 cm | 1.6 cm | 1.6 cm |
| batch3 | 10 | 1.5 cm | 1.4 cm | 2.4 cm | 2.4 cm |
