# ShooterManager method quick-reference

Purpose: handle flywheel spin-up, feed motors, gates, pushers, and LED status for firing artifacts.

## Public API
- `ShooterManager(HardwareConfig hw, Telemetry tel)`: configure motors/servos/LEDs to safe defaults, set feed motor idle power, and store telemetry.
- `void setTargetRPM(double rpm)`: command flywheel velocity (ticks/sec) and set hood servo angle based on rpm heuristic.
- `void waitUntilReady()`: block for up to ~2s while flywheel reaches target velocity, reporting telemetry feedback.
- `void setLedReady(boolean ready)`: set Blinkin pattern (heartbeat green when ready; orange/red when not).
- `void update()`: poll flywheel exit distance sensor to detect when a shot leaves; updates `shotJustExited` flag.
- `boolean shotJustExited()`: true for the frame immediately after the exit sensor transitions from blocked to clear.
- `void fireOneFromChannel(int channel)`: open the selected gate (0=left, 1=right), pulse its pusher, then close gate/pusher.
- `void stopAll()`: power down flywheel/feed, close both gates/pushers, and set LED to not-ready.

## Notes for tele-op integration
- Call `setTargetRPM` before firing cycles; pair `waitUntilReady` and `setLedReady(true)` for operator cues.
- Invoke `update()` each loop if you want to react to `shotJustExited()` events (e.g., decrement inventory or trigger next feed).
- Constants `PUSHER_TIME_MS`, `FEED_IDLE_POWER`, and `EXIT_BLOCKED_MM` can be tuned for your hardware.
