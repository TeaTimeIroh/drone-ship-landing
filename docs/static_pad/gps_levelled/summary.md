# Landing trials

- Attempts: 30, landed and measured: 30, failed: 0
- Touchdown error from the pad centre: median 1.3 cm, mean 1.2 cm, 95th percentile 2.1 cm, worst 2.2 cm
- Inside the 1.0 m marker: 30/30; on the 1.25 m pad: 30/30
- Average miss direction: -0.5 cm east, 0.3 cm north (a consistent offset here points to a systematic cause)
- Time from start point to touchdown: mean 16.7 s, longest 17.5 s (simulation time)
- Detector range / true height of the drone above the pad at the start point: mean 1.008 (range 1.005 to 1.012, 30 readings). The camera hangs roughly 0.1 m below the point Gazebo reports, so a perfect detector would read slightly under 1.

| Batch | Landings | Median | Mean | 95th pct | Worst |
|---|---|---|---|---|---|
| batch1 | 10 | 1.2 cm | 1.1 cm | 2.2 cm | 2.2 cm |
| batch2 | 10 | 1.3 cm | 1.2 cm | 1.8 cm | 1.8 cm |
| batch3 | 10 | 1.1 cm | 1.1 cm | 2.1 cm | 2.1 cm |
