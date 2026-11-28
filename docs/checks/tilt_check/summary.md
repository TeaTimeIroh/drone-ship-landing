# Tilt check

The correction as written is the best variant. While tilted it cuts the error from 1.13 m to 0.05 m.

- Detections used: 308, of which 127 with more than 4 degrees of tilt and 99 with less than 1.5 degrees. Largest tilt: 20.5 degrees.
- Heading: 89.9 degrees from east according to the autopilot, 89.3 degrees according to the camera (these should agree).
- Best time offset when looking up the pose for an image: +0.00 s (error 0.052 m there, 0.052 m at zero).

Horizontal error against the position the autopilot reports (RMS):

| Conversion | All detections | Tilted | Level |
|---|---|---|---|
| raw, tilt ignored | 0.741 m | 1.131 m | 0.096 m |
| levelled, as published | 0.052 m | 0.055 m | 0.050 m |
| levelled, roll reversed | 0.987 m | 1.505 m | 0.128 m |
| levelled, pitch reversed | 0.964 m | 1.472 m | 0.110 m |
| levelled, both reversed | 1.378 m | 2.103 m | 0.161 m |

The autopilot's own position error is included in every number above.
