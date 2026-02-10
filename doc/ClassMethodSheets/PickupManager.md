# PickupManager method quick-reference

Purpose: manage the bottom intake and classify artifacts entering each funnel using color sensors and auger nudges.

## Public API
- `PickupManager(HardwareConfig hw, Telemetry tel, BallInventory inv)`: store hardware, telemetry, and inventory; starts intake roller and both augers running.
- `void update()`: poll bottom color sensors, detect new arrivals with hysteresis, classify color, push into `BallInventory`, nudge the respective auger, and log telemetry.

## Notes for tele-op integration
- Call `update()` repeatedly while intaking to keep inventory in sync; it does not manage gates or central pushers.
- Presence threshold and nudge duration are tuned via `PRESENCE_THRESHOLD` and `AUGER_NUDGE_MS` constants if you need faster/slower response.
