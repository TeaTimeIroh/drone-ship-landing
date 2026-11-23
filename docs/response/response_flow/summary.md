# Response to velocity commands: response_flow

| Command | Seen by | Half speed after | 90% (start) or 10% (stop) after | Peak or reverse speed | Moved in 5 s |
|---|---|---|---|---|---|
| start east, 1.0 m/s | Gazebo (truth) | 0.66 s | 0.96 s | +1.08 m/s | 4.31 m |
| stop east, 1.0 m/s | Gazebo (truth) | 0.63 s | 0.99 s | -0.07 m/s | 0.65 m |
| start north, 1.0 m/s | Gazebo (truth) | 0.60 s | 0.90 s | +1.07 m/s | 4.35 m |
| stop north, 1.0 m/s | Gazebo (truth) | 0.63 s | 0.99 s | -0.07 m/s | 0.68 m |
| start east, 1.0 m/s | autopilot | 0.66 s | 0.93 s | +1.19 m/s | 4.29 m |
| stop east, 1.0 m/s | autopilot | 0.66 s | 0.99 s | -0.08 m/s | 0.67 m |
| start north, 1.0 m/s | autopilot | 0.60 s | 0.96 s | +1.15 m/s | 4.32 m |
| stop north, 1.0 m/s | autopilot | 0.63 s | 1.02 s | -0.11 m/s | 0.70 m |

- The autopilot's east speed runs +0.05 s behind the true speed (negative means ahead).
- The autopilot's north speed runs +0.05 s behind the true speed (negative means ahead).
- True height during the recording: 4.85 to 6.54 m (above the Gazebo ground, at the drone's centre).
