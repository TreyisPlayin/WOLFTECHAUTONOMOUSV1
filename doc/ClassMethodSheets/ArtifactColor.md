# ArtifactColor quick-reference

Enum defining known artifact colors for the game: `PURPLE`, `GREEN`, `UNKNOWN`.

## Notes for tele-op integration
- Use as the canonical color type when communicating between `PickupManager`, `BallInventory`, `HuskySuggestor`, and `ShooterManager`.
- Map HuskyLens block IDs to these values in `HuskySuggestor` and maintain channel assignments using `BallInventory` helpers.
