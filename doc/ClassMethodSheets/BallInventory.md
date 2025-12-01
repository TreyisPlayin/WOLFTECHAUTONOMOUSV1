# BallInventory method quick-reference

Purpose: track artifact colors in two channels (left/right) with two slots each, ordered front→back.

## Public API
- `BallInventory()`: initializes all slots to `UNKNOWN` and sets active channel to 0.
- `void clearAll()`: reset every slot to `UNKNOWN` for both channels.
- `void setActiveChannel(int channel)`: select which channel methods like `seedPreset` operate on (ignored if out of range).
- `int getActiveChannel()`: return the currently selected channel index.
- `void seedPreset(ArtifactColor[] preset)`: load a preset sequence into the active channel’s slots (extra slots become `UNKNOWN`).
- `void insertAtBack(int channel, ArtifactColor color)`: push a color into the back of the specified channel, shifting existing items toward the front.
- `void popFront(int channel)`: remove the front-most slot of the channel, shifting the rest forward and marking the back as `UNKNOWN`.
- `ArtifactColor peekFront(int channel)`: inspect the color at the front of the channel; returns `UNKNOWN` on invalid channel.
- `ArtifactColor[][] getSnapshot()`: copy of all channel slots for telemetry or debugging.

## Notes for tele-op integration
- Combine with `PickupManager` when logging intakes per side; feed `peekFront` into `ShooterManager` channel selection logic.
- Channel indices: `0` = left, `1` = right; slots index 0 is the firing/front position.
