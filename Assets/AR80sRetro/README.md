# AR 80s Retro MVP Scaffold

This folder contains the first Unity-side implementation for the MVP described in the project plan.

## Implemented

- Detection result data contract for YOLO-style output.
- Scriptable replacement rules from detected labels to retro prefabs.
- AR Foundation raycast-based 2D bbox center to AR world pose solver.
- Replacement manager with confidence gating, multi-frame confirmation, smoothing, duplicate suppression, and lost-object grace time.
- Runtime AR anchors, position dead-zone, confirmed relocation, and locked post-placement rotation/scale.
- Mock input for testing the placement loop before a real detector is connected.
- Detector stub with a throttled inference interval.

## Unity scene setup

1. Create a Unity 2022.3 LTS Android project.
2. Install `AR Foundation` and `ARCore XR Plugin`.
3. Add an `AR Session`, `XR Origin`, `AR Camera`, `AR Plane Manager`, and `AR Raycast Manager`.
4. Add `ARRaycastPositionSolver`, `RetroReplacementManager`, and `RetroDetectionPipeline` to scene objects.
5. Create a `RetroPrefabLibrary` asset and add rules such as `cup -> enamel cup prefab`.
6. For early testing, add `MockDetectionInput` and hold Space in Play Mode while an AR plane is visible.

## Stability tuning

The replacement manager automatically adds an `ARAnchorManager` to the active XR Origin when needed.
Its scene defaults ignore position changes below 4 cm, require three consistent measurements before
relocating, reject jumps above 75 cm, and keep a replacement alive for three seconds after detection
is lost. Rotation and scale are fixed when the replacement is first created.

## Next implementation step

Replace `YoloObjectDetector` with a real detector backend. Keep its output as `DetectionResult` values with normalized top-left-origin bounding boxes so the placement and replacement layers remain unchanged.
