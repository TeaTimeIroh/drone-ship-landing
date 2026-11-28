# Moving pad landing trials

- Pad motion: line at 0.5 m/s
- Attempts: 19, landed on the pad and measured: 17, touched down off the pad: 0, failed: 2
- Failure reasons: Moving pad landing failed: the pad was not found
- Touchdown error from the pad centre: median 2.4 cm, mean 4.5 cm, 95th percentile 30.3 cm, worst 30.3 cm
- More than 15 cm off: 1 of 17
- Along the pad's direction of travel (17 landings): on average 4.1 cm ahead of the centre, spread 7.0 cm. Across it: on average 0.1 cm to the left, spread 0.8 cm
- Landings where the pad held its velocity between commitment and touchdown: 15, median 2.1 cm, worst 6.1 cm
- Landings where the pad changed its velocity between commitment and touchdown: 2, median 19.8 cm, worst 30.3 cm
- The drone's own estimate of its offset at touchdown differed from the truth by 2.1 cm (median)
- Time from the start point to touchdown: mean 23.8 s, longest 34.9 s (simulation time); final descents started per landing: 1.0
- Motors stopped 2.1 s after contact on average; LAND mode fallback used 0 times

| Batch | Attempts | Landed | Median | Mean | 95th pct | Worst |
|---|---|---|---|---|---|---|
| batch1 | 6 | 5 | 1.3 cm | 2.9 cm | 9.3 cm | 9.3 cm |
| batch2 | 3 | 2 | 1.6 cm | 1.6 cm | 2.0 cm | 2.0 cm |
| batch3 | 10 | 10 | 3.0 cm | 5.9 cm | 30.3 cm | 30.3 cm |
