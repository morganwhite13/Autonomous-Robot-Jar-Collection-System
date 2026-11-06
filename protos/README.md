# Robot protos

Both worlds load the robot from local files in this folder:

```
EXTERNPROTO "../protos/Pioneer3dx.proto"
EXTERNPROTO "../protos/Pioneer3Gripper.proto"
```

The original project never committed these, so nobody else could open the world. The files here are **reconstructions**: Cyberbotics' own Pioneer 3-DX protos from **Webots R2021b**, unchanged except for one added `EXTERNPROTO` line. `tools/fetch-protos.sh` shows exactly how they were made.

## Why R2021b

The world uses the old `NUE` coordinate system (Y up). Today's Webots Pioneer protos are `ENU` (Z up), so they would load lying on their side. The R2021b protos are `NUE` and match everything the controllers expect:

| What the controllers expect | Webots R2021b protos |
|---|---|
| sonar names `so0`, `so1`, `so3`, `so4`, `so6`, `so7` | `so0` to `so15` |
| `lift motor` range -0.0499 to 0.001 | `minPosition -0.05`, `maxPosition 0.05` |
| finger motors 0.01 to 0.099 | `maxPosition 0.1` |
| wheel radius 9.75 cm, axle 33 cm (used in the tests' kinematic model) | radius 0.0975, wheel anchors at +/-0.165 |
| `Pioneer3dx` fields the world sets (`translation`, `rotation`, `name`, `controller`, `supervisor`, `extensionSlot`) | all present |

## Keep the `R2021b` header

The first line must stay `#VRML_SIM R2021b utf8`. Webots R2022a changed two conventions: a `Cylinder`'s axis went from Y to Z, and a `Camera` now looks along its x axis instead of -z. When Webots R2025a loads a file that says R2021b, it converts both (the log shows "Trying to resolve the backwards compatibility by adjusting the rotation"). This also covers the camera and touch sensor that the world adds to the robot's `extensionSlot`.

An earlier version of these files had the header changed to R2025a. Webots then skipped the conversion, and two things broke in a way that looked like controller bugs:

| Header | Wheels | Camera |
|---|---|---|
| R2025a (wrong) | The wheel collision cylinders lay across the wheel instead of along the axle. Every half turn the robot dropped onto its caster fork and stopped for about 2.5 s while the wheels kept spinning. | Pointed out of the robot's right side, rolled 90 degrees: jars were seen while beside the robot, and every approach drove away from them. |
| R2021b (correct) | Constant speed (6.1 cm/s at the approach speed), body level. | Faces forward, upright: jars appear at the bearings the world file puts them. |

## How far they have been checked

- **Loaded and driven in Webots R2025a** (Linux, headless) by `tools/run-in-webots.sh`. Webots converts the R2021b content as described above, and the robot drives, turns, scans and grabs as the controllers expect. v2 delivers all five jars on it; see [the Webots results](../docs/BEFORE_AND_AFTER.md#measured-in-webots).
- **Checked statically against Webots R2025a.** Every node type the three files use (40 in total) exists in R2025a's node definitions (`resources/nodes/*.wrl`), and so does every field they set. R2025a's `WorldInfo` still accepts `coordinateSystem "NUE"`. All 19 `webots://` texture paths exist in the R2025a release.
- They are still reconstructions. If you have the originals from the project folder you first ran it in, those are the ground truth.

## License

The files keep their original header: "Copyright Cyberbotics Ltd. Licensed for use only with Webots." This repository is a Webots project and uses them only with Webots. They are excluded from the repository's MIT license.
