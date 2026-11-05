# Robot protos (not included)

Both worlds load the robot from two local files that were never committed to this repository:

```
EXTERNPROTO "../protos/Pioneer3dx.proto"
EXTERNPROTO "../protos/Pioneer3Gripper.proto"
```

Webots cannot open `worlds/ProjectWorld2025.wbt` or `worlds/ProjectWorld2025_v2.wbt` until both files are here.

## Option 1 (recommended): copy your original files

If you still have the project folder you ran in Webots, copy `Pioneer3dx.proto` and `Pioneer3Gripper.proto` (plus any `.proto` files they reference, such as `Pioneer3DistanceSensor.proto`) into this folder. Then check the licence header before committing them: Cyberbotics protos say "Licensed for use only with Webots".

## Option 2 (untested): rebuild them from upstream Webots

The world uses the old `NUE` coordinate system (Y up). Today's Webots Pioneer protos are `ENU` (Z up), so they would load lying on their side. The protos that shipped with **Webots R2021b** are `NUE` and match everything the controllers expect:

| What the controllers expect | Webots R2021b protos |
|---|---|
| sonar names `so0`, `so1`, `so3`, `so4`, `so6`, `so7` | `so0` to `so15` |
| `lift motor` range -0.0499 to 0.001 | `minPosition -0.05`, `maxPosition 0.05` |
| finger motors 0.01 to 0.099 | `maxPosition 0.1` |
| wheel radius 9.75 cm, axle 33 cm (used in the tests' kinematic model) | radius 0.0975, wheel anchors at +/-0.165 |

`tools/fetch-protos.sh` downloads those three files and updates their headers for R2025a. **It has not been tested in Webots**: Webots could not be launched in the environment where this version was written. If it doesn't work, use Option 1.
