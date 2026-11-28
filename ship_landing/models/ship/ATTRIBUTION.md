# Attribution

The hull mesh (`meshes/Boat05.dae`) and its four textures (`materials/textures/Boat05_*`) are from
the model **"Vessel E" by OpenRobotics**, published on Gazebo Fuel:
https://app.gazebosim.org/OpenRobotics/fuel/models/Vessel%20E

They are licensed under Creative Commons Attribution 4.0 International
(https://creativecommons.org/licenses/by/4.0/) and are included here unchanged.

`model.sdf` is this project's own and differs from the original model: the boat hangs from
the world on joints instead of being a free body, the mesh is used for looks only, the shapes
the drone can touch are plain boxes, and a landing pad is added on the aft deck.
`materials/textures/aruco_pad.png` is this project's marker, not part of "Vessel E".
