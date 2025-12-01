# PatternAuto method quick-reference

Purpose: sample autonomous state machine that scores a motif (PURPLE, GREEN, PURPLE) using pickup/shooter/HuskyLens helpers.

## Public API
- `void runOpMode()`: full LinearOpMode entry point—initializes hardware, localization, managers, then steps through state machine (drive to preload, score, search for pickups with HuskyLens suggestions, pick up, drive to goal, score, finish). Uses helper methods below.

## Internal helpers (structure reference)
- `doDriveToPreloadSpot()`: timed forward move placeholder.
- `doScorePreload()`: spin shooter to target RPM, mark LED ready, fire motif colors, and stop shooter.
- `doLookingForPickup()`: request needed color, query `HuskySuggestor`, drive/strafe toward blocks or rotate to search, return true when aligned/close.
- `doPickingUp()`: run `pickup.update()`, check for needed color, advance motif index when acquired.
- `doDriveToGoal()`: timed backward move placeholder.
- `doScoreMotif()`: re-fire motif contents at goal with shooter spin-up/LED cues.
- `getNeededColor()`: choose current motif color (placeholder; swap for pickup-driven logic as needed).
- `driveMecanum(drive, strafe, turn)`: power mecanum wheels with normalization.
- `stopDrive()`: convenience wrapper for zeroing drive power.

## Notes for tele-op integration
- Use as a reference for sequencing managers rather than running directly in tele-op; helper patterns show how to call shooter/pickup/HuskySuggestor together.
