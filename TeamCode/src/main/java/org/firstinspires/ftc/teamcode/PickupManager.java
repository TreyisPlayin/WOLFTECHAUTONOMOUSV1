package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PickupManager — bottom intake using color sensors for presence + classification.
 * Assumes:
 *  • intakeRoller (CR) always on
 *  • leftAuger/rightAuger (CR) always on
 *  • bottomColorLeft / bottomColorRight detect entries at each funnel bottom
 *
 * NOTE: central pusher + top analog sensors can be integrated later to
 * throttle/route, but this manager simply records what arrives at each funnel
 * and nudges it forward with augers that are already spinning.
 */
public class PickupManager {

    private final HardwareConfig hw;
    private final Telemetry telemetry;
    private final BallInventory inventory;

    // presence hysteresis at funnel bottoms
    private boolean lastPresentLeft = false;
    private boolean lastPresentRight = false;

    // tune
    private static final int PRESENCE_THRESHOLD = 80;
    private static final long AUGER_NUDGE_MS = 150;

    private final ElapsedTime t = new ElapsedTime();

    public PickupManager(HardwareConfig hw, Telemetry tel, BallInventory inv) {
        this.hw = hw;
        this.telemetry = tel;
        this.inventory = inv;

        // Always-on devices per your request
        if (hw.intakeRoller != null) hw.intakeRoller.setPower(1.0);
        if (hw.leftAuger != null)    hw.leftAuger.setPower(1.0);
        if (hw.rightAuger != null)   hw.rightAuger.setPower(1.0);
    }

    private boolean present(ColorSensor s) {
        if (s == null) return false;
        return (s.red() + s.green() + s.blue()) > PRESENCE_THRESHOLD;
    }

    private ArtifactColor classify(ColorSensor s) {
        if (s == null) return ArtifactColor.UNKNOWN;
        int r = s.red(), g = s.green(), b = s.blue();
        int sum = r + g + b;
        if (sum < PRESENCE_THRESHOLD) return ArtifactColor.UNKNOWN;
        return (g > r && g > b) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;
    }

    private void nudge(CRServo auger) {
        if (auger == null) return; // augers are already running; brief extra is optional
        double prev = 1.0;
        auger.setPower(1.0); // keep full; this method remains for symmetry
        t.reset();
        while (t.milliseconds() < AUGER_NUDGE_MS) { /* brief wait */ }
        auger.setPower(prev);
    }

    /** Call every loop while intaking. */
    public void update() {
        boolean presentL = present(hw.bottomColorLeft);
        boolean presentR = present(hw.bottomColorRight);

        if (presentL && !lastPresentLeft) {
            ArtifactColor color = classify(hw.bottomColorLeft);
            inventory.insertAtBack(0, color);
            nudge(hw.leftAuger);
            telemetry.addData("Pickup", "LEFT new %s", color);
        }

        if (presentR && !lastPresentRight) {
            ArtifactColor color = classify(hw.bottomColorRight);
            inventory.insertAtBack(1, color);
            nudge(hw.rightAuger);
            telemetry.addData("Pickup", "RIGHT new %s", color);
        }

        lastPresentLeft = presentL;
        lastPresentRight = presentR;
    }
}
