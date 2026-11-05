# Robot protos

Both worlds load the robot from local files in this folder:

```
EXTERNPROTO "../protos/Pioneer3dx.proto"
EXTERNPROTO "../protos/Pioneer3Gripper.proto"
```

The original project never committed these, so nobody else could open the world. The files here are **reconstructions**: Cyberbotics' own Pioneer 3-DX protos from **Webots R2021b**, with their headers updated for R2025a. `tools/fetch-protos.sh` shows exactly how they were made.

## Why R2021b

The world uses the old `NUE` coordinate system (Y up). Today's Webots Pioneer protos are `ENU` (Z up), so they would load lying on their side. The R2021b protos are `NUE` and match everything the controllers expect:

| What the controllers expect | Webots R2021b protos |
|---|---|
| sonar names `so0`, `so1`, `so3`, `so4`, `so6`, `so7` | `so0` to `so15` |
| `lift motor` range -0.0499 to 0.001 | `minPosition -0.05`, `maxPosition 0.05` |
| finger motors 0.01 to 0.099 | `maxPosition 0.1` |
| wheel radius 9.75 cm, axle 33 cm (used in the tests' kinematic model) | radius 0.0975, wheel anchors at +/-0.165 |
| `Pioneer3dx` fields the world sets (`translation`, `rotation`, `name`, `controller`, `supervisor`, `extensionSlot`) | all present |

## How far they have been checked

- **Checked statically against Webots R2025a.** Every node type the three files use (40 in total) exists in R2025a's node definitions (`resources/nodes/*.wrl`), and so does every field they set. R2025a's `WorldInfo` still accepts `coordinateSystem "NUE"`. All 19 `webots://` texture paths exist in the R2025a release.
- **Not yet loaded in Webots.** Webots could not be launched in the environment where they were added. If the robot doesn't appear or behaves oddly, replace these files with the originals from the project folder you first ran it in (plus any `.proto` they reference).

## Licence

The files keep their original header: "Copyright Cyberbotics Ltd. Licensed for use only with Webots." This repository is a Webots project and uses them only with Webots.
