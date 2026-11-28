# Landing trials

- Attempts: 30, landed and measured: 30, failed: 0
- Touchdown error from the pad centre: median 2.2 cm, mean 2.3 cm, 95th percentile 3.0 cm, worst 3.2 cm
- Inside the 1.0 m marker: 30/30; on the 1.25 m pad: 30/30
- Average miss direction: -1.4 cm east, -1.6 cm north (a consistent offset here points to a systematic cause)
- Time from start point to touchdown: mean 17.2 s, longest 20.7 s (simulation time)
- Detector range / true height of the drone above the pad at the start point: mean 1.001 (range 0.998 to 1.003, 30 readings). The camera hangs roughly 0.1 m below the point Gazebo reports, so a perfect detector would read slightly under 1.

| Batch | Landings | Median | Mean | 95th pct | Worst |
|---|---|---|---|---|---|
| batch1 | 10 | 2.1 cm | 2.2 cm | 2.9 cm | 2.9 cm |
| batch2 | 10 | 2.5 cm | 2.3 cm | 3.0 cm | 3.0 cm |
| batch3 | 10 | 2.2 cm | 2.3 cm | 3.2 cm | 3.2 cm |
