# Moving pad landing trials

- Pad motion: line at 0.5 m/s
- Attempts: 19, landed on the pad and measured: 17, touched down off the pad: 0, failed: 2
- Failure reasons: Moving pad landing failed: the pad was not found
- Touchdown error from the pad centre: median 8.8 cm, mean 13.2 cm, 95th percentile 27.6 cm, worst 27.6 cm
- More than 15 cm off: 7 of 17
- Along the pad's direction of travel (17 landings): on average 12.3 cm ahead of the centre, spread 12.1 cm. Across it: on average 0.3 cm to the left, spread 1.0 cm
- Landings where the pad held its velocity between commitment and touchdown: 9, median 3.2 cm, worst 27.6 cm
- Landings where the pad changed its velocity between commitment and touchdown: 8, median 10.9 cm, worst 26.7 cm
- The drone's own estimate of its offset at touchdown differed from the truth by 5.6 cm (median)
- Time from the start point to touchdown: mean 21.1 s, longest 36.1 s (simulation time); final descents started per landing: 1.0
- Motors stopped 2.0 s after contact on average; LAND mode fallback used 0 times

| Batch | Attempts | Landed | Median | Mean | 95th pct | Worst |
|---|---|---|---|---|---|---|
| batch1 | 6 | 5 | 1.8 cm | 8.8 cm | 26.2 cm | 26.2 cm |
| batch2 | 3 | 2 | 14.2 cm | 14.2 cm | 26.7 cm | 26.7 cm |
| batch3 | 10 | 10 | 15.7 cm | 15.2 cm | 27.6 cm | 27.6 cm |
