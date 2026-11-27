# Camera-only pad tracking against the truth: relative_check

- Recording: 39 s, pad in view (seen in the last 0.5 s) for 97% of it
- True relative speed between pad and drone: up to 1.37 m/s

| Height above the pad | Share of the time | Position error: typical, 95% | Velocity error: typical, 95% | Height error: average |
|---|---|---|---|---|
| all heights | 100% | 8.3 cm, 22.1 cm | 7.8 cm/s, 15.9 cm/s | +8.3 cm |
| above 2 m | 49% | 11.8 cm, 24.1 cm | 10.8 cm/s, 19.0 cm/s | +13.7 cm |
| 0.6 to 2 m | 11% | 2.4 cm, 4.7 cm | 6.0 cm/s, 13.8 cm/s | +12.5 cm |
| below 0.6 m | 40% | 0.3 cm, 0.5 cm | 0.6 cm/s, 1.6 cm/s | +0.6 cm |

- The tracker's velocity runs +0.10 s behind the true one (negative means ahead).
- "Typical" is the root mean square. Position and velocity are of the pad relative to the drone, in the horizontal plane.
- Height error is the tracker's height minus the true height of the drone's centre above the pad.
