# HuskySuggestor method quick-reference

Purpose: translate HuskyLens COLOR_RECOGNITION blocks into drive/strafe suggestions toward a desired artifact color.

## Public API
- `HuskySuggestor(HardwareConfig hw, Telemetry telemetry)`: cache HuskyLens instance from hardware config and telemetry sink.
- `Suggestion suggestForColor(ArtifactColor desiredColor)`: scan HuskyLens blocks, pick the largest block matching the trained ID for the requested color, and return drive/strafe commands plus debug info. Adds telemetry notes when HuskyLens is missing, untrained, or lacks blocks.

## Suggestion fields
- `hasTarget`: true when a matching block was found.
- `drive`: forward command (positive drives toward the block until it appears wide enough).
- `strafe`: lateral command (positive moves right) proportional to pixel error from image center.
- `blockId`: HuskyLens ID chosen.
- `errorX`: pixel offset from image center (positive to the right).
- `widthPx`: detected block width in pixels.

## Notes for tele-op integration
- Combine with your drive mixer: blend `drive`/`strafe` with manual controls or let auto logic drive when `hasTarget` is true.
- Tune constants (`CENTER_DEADBAND_PX`, `CLOSE_WIDTH_PX`, `MAX_DRIVE`, `MAX_STRAFE`, `STRAFE_K`) if the robot overshoots or responds too slowly.
