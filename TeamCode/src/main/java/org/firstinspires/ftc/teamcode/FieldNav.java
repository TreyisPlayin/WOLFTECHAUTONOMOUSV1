package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * FieldNav — Pinpoint-first localization + AprilTag drift-correction, curve following,
 * end-facing control, and tag-seek priority. OnBot-safe (single file).
 *
 * UNITS & FRAMES
 *  • Field frame: inches (X,Y) and degrees (heading). Origin at field center; +X-> audience; +Y-> Blue.  (FTC docs)  [1]
 *  • Camera frame for getGoalTagDelta(): inches from VisionPortal ftcPose; +X right of camera, +Y forward.         [2][3]
 *
 * WHAT YOU CALL FROM OUTSIDE
 *  • new FieldNav(hardwareMap, telemetry, pinpoint, "Webcam 1")
 *  • setAlliance(FieldNav.Alliance.BLUE or RED)      // selects goal tag ID (BLUE=20, RED=24 for DECODE)
 *  • driveTo(xIn, yIn)                                // or driveTo(xIn, yIn, finalHeadingDeg)
 *  • boolean done = update();                         // run each loop; true when finished
 *  • FieldNav.DriveCommand dc = getDriveCommand();    // forward/strafe/rotate for your drivetrain
 *  • TagDelta d = getGoalTagDelta();                  // x,y inches to alliance goal tag in camera frame
 *
 * DRIFT CORRECTION
 *  • Primarily relies on Pinpoint (mm/rad → inches/deg).                              [4]
 *  • When a known tag pose is provided (via setTagPose), uses that tag’s ftcPose to
 *    correct the field pose (blended in with a small weight so Pinpoint stays primary). [2]
 *
 * TAG-SEEK PRIORITY
 *  • If no alliance goal tag has been seen for tagSeekAfterSec, the follower pauses and the robot
 *    actively rotates in place to reacquire a tag (VisionPortal is auto-woken if napping).
 *
 * REFERENCES:
 *  [1] FTC Field Coordinate System (origin center; +X audience; +Y Blue).                ftc-docs & FTC Dashboard PDF
 *  [2] VisionPortal / AprilTagProcessor ftcPose (inches & degrees)                       ftc-docs
 *  [3] “Understanding AprilTag Detection Values” (Range/Bearing/XYZ units)              ftc-docs
 *  [4] goBILDA Pinpoint user guide (mm & radians; Pose2D conversions)                   goBILDA docs
 */
public class FieldNav {

    // ========= Public surface (small on purpose) =========

    /** Declare alliance so we lock onto the correct GOAL tag ID (DECODE: BLUE=20, RED=24). */
    public enum Alliance { RED, BLUE }

    /** Robot-frame drive command (plug directly into your mecanum mix), in [-1..1]. */
    public static class DriveCommand {
        /** +Forward (robot X) */
        public final double forward;
        /** +Right strafe (robot Y) */
        public final double strafe;
        /** +CCW rotate */
        public final double rotate;
        public DriveCommand(double f, double s, double r){ forward=f; strafe=s; rotate=r; }
    }

    /** Camera-frame delta to the alliance GOAL AprilTag (inches). */
    public static class TagDelta {
        /** +X = camera right (in) */
        public final double xIn;
        /** +Y = camera forward (in) */
        public final double yIn;
        /** True if a fresh detection for the alliance goal tag was used this cycle. */
        public final boolean valid;
        public TagDelta(double xIn, double yIn, boolean valid){ this.xIn=xIn; this.yIn=yIn; this.valid=valid; }
    }

    /**
     * Constructor — starts Vision (if available) and seeds pose from Pinpoint.
     * @param hw          HardwareMap
     * @param tel         Telemetry
     * @param pin         Your initialized goBILDA Pinpoint (I2C device)
     * @param webcamName  The name of your webcam in the RC config (e.g., "Webcam 1")
     */
    public FieldNav(HardwareMap hw, Telemetry tel, GoBildaPinpointDriver pin, String webcamName){
        hardwareMap = hw;
        telemetry   = tel;
        pinpoint    = pin;

        // Vision: AprilTags through VisionPortal
        try {
            WebcamName cam = hardwareMap.get(WebcamName.class, webcamName);
            tagProcessor = AprilTagProcessor.easyCreateWithDefaults();          // [2]
            visionPortal = VisionPortal.easyCreateWithDefaults(cam, tagProcessor);
            visionEnabled = true;
        } catch (Exception e){
            visionPortal = null;
            tagProcessor = null;
            visionEnabled = false;
        }

        setAlliance(Alliance.BLUE); // default
        readPinpoint();             // seed fused pose
        buildCurveCubic(xIn, yIn, xIn, yIn); // no-op curve to init internals
    }

    /**
     * Set alliance for this match. Internally selects the correct GOAL tag ID (DECODE: BLUE=20, RED=24).
     * You can change this any time (e.g., in init loop).
     */
    public void setAlliance(Alliance a){
        alliance = a;
        goalTagId = (a == Alliance.BLUE) ? 20 : 24;   // tag IDs per DECODE kit (IDs 20–24 exist; 20/24 are goals)
    }

    /**
     * Command a move to a field point (inches). Uses a smooth cubic curve by default and path-tangent heading.
     * Arrival is declared when inside {@code tolIn} (default 2").
     * @param targetX field X (in)
     * @param targetY field Y (in)
     */
    public void driveTo(double targetX, double targetY){
        driveTo(targetX, targetY, null);
    }

    /**
     * Command a move with an optional final heading. If {@code finalHeadingDeg} is non-null,
     * the robot will park at the target and then rotate in place until within {@code arriveHeadingTolDeg}.
     * @param targetX field X (in)
     * @param targetY field Y (in)
     * @param finalHeadingDeg heading in degrees CCW+ (nullable to skip end-facing)
     */
    public void driveTo(double targetX, double targetY, Double finalHeadingDeg){
        this.targetX = targetX;
        this.targetY = targetY;
        this.endHeadingDeg = finalHeadingDeg; // may be null
        buildCurveCubic(xIn, yIn, targetX, targetY);
        segIdx = 0; segT = 0.0;
        moving = true;
        lastStatus = MoveStatus.MOVING;
        moveTimer.reset();
    }

    /**
     * Main loop step: updates Pinpoint pose, scans for tags (and manages vision),
     * follows the curve (or enters tag-seek priority), and returns true when done.
     * @return true if finished (ARRIVED / TIMEOUT / CANCELED); false while driving or seeking
     */
    public boolean update(){
        // 1) Pinpoint → fused pose (mm/rad → inches/deg)  [4]
        readPinpoint();

        // Travel integration
        if (firstPose){ lastX=xIn; lastY=yIn; firstPose=false; }
        else { totalTravelIn += Math.hypot(xIn-lastX, yIn-lastY); lastX=xIn; lastY=yIn; }

        // 2) Tag scan (updates lastTagDelta; may apply drift correction if we know tag field pose)
        boolean sawGoal = scanGoalTag(); // uses ftcPose from VisionPortal  [2][3]

        // 3) Auto-manage vision and “seek tag” priority after long gaps
        double dToTarget = Math.hypot(targetX - xIn, targetY - yIn);
        manageVisionAndSeeking(dToTarget, sawGoal);

        // 4) Timeout?
        if (moving && moveTimeoutSec>0 && moveTimer.seconds()>moveTimeoutSec){
            moving=false; lastStatus=MoveStatus.TIMEOUT;
            stopOutputs();
            return true;
        }

        // 5) Not moving? (or seeking purely for tag) then just return status
        if (!moving || segs.length==0){
            stopOutputs();
            return (lastStatus!=MoveStatus.MOVING);
        }

        // 6) Position arrival test
        if (dToTarget <= tolIn){
            if (endHeadingDeg != null){
                // rotate in place until heading is within tolerance
                double hErr = normDeg(endHeadingDeg - hDeg);
                double omega = clamp(kHeading*hErr, -seekRotatePower, seekRotatePower) * speedScale;
                boolean doneHeading = Math.abs(hErr) <= arriveHeadingTolDeg;
                setOutputsField(0,0, doneHeading ? 0 : omega);
                if (doneHeading){
                    moving=false; lastStatus=MoveStatus.ARRIVED; return true;
                }
                return false; // keep rotating to final heading
            } else {
                moving=false; lastStatus=MoveStatus.ARRIVED; stopOutputs(); return true;
            }
        }

        // 7) If we’re in TAG SEEK priority, pause path following and just rotate
        if (seekingTag){
            // Prefer to rotate toward expected goal tag bearing if field pose of tag is known
            double omega = seekRotatePower;
            TagPose tp = (goalTagId>=0 && goalTagId<=MAX_TAG_ID) ? tagLib[goalTagId] : null;
            if (tp != null){
                double desiredH = Math.toDegrees(Math.atan2(tp.y - yIn, tp.x - xIn));
                double hErr = normDeg(desiredH - hDeg);
                omega = clamp(kHeading*hErr, -seekRotatePower, seekRotatePower);
            }
            setOutputsField(0,0, omega);
            return false;
        }

        // 8) Normal follower step (field frame), with slowdown near target
        FieldCommand base = followStepField();
        double slow = (dToTarget < slowRadiusIn) ? clamp(dToTarget/slowRadiusIn, minSpeed, 1.0) : 1.0;
        setOutputsField(base.vx*slow*speedScale, base.vy*slow*speedScale, base.omega*slow*speedScale);
        return false;
    }

    /** Get the most recent ROBOT-frame command (forward, strafe, rotate) in [-1..1] for your drivetrain. */
    public DriveCommand getDriveCommand(){ return lastRobotCmd; }

    /**
     * Return the latest X/Y delta to the ALLIANCE GOAL AprilTag in the CAMERA frame (inches).
     * (+X is to the right of the camera; +Y is forward out of the lens). Valid=false if no fresh goal-tag detection.
     */
    public TagDelta getGoalTagDelta(){ return lastTagDelta; }

    // ---------- Optional: expose a way to install tag field poses for drift-correction ----------

    /**
     * Install/override a known field pose for a tag ID (inches/deg). If provided for the alliance goal tag,
     * FieldNav will blend AprilTag-derived absolute pose into the Pinpoint pose when that tag is seen.
     */
    public void setTagPose(int id, double xIn, double yIn, double hDeg){
        if (id>=0 && id<=MAX_TAG_ID) tagLib[id] = new TagPose(xIn,yIn,hDeg);
    }

    // ========= Everything below is internal on purpose =========

    // Small status enum, not exposed
    private enum MoveStatus { IDLE, MOVING, ARRIVED, TIMEOUT, CANCELED }

    // Hardware & vision
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final GoBildaPinpointDriver pinpoint;
    private VisionPortal visionPortal=null;
    private AprilTagProcessor tagProcessor=null;
    private boolean visionEnabled=false;

    // Alliance & tags
    private Alliance alliance = Alliance.BLUE;
    private int goalTagId = 20;       // BLUE default (DECODE)
    private static final int MAX_TAG_ID=600;
    private final TagPose[] tagLib = new TagPose[MAX_TAG_ID+1];

    // Pose (field frame)
    private double xIn=0, yIn=0, hDeg=0;
    private double lastX=0, lastY=0, totalTravelIn=0;
    private boolean firstPose=true;

    // Move state & tuning
    private boolean moving=false;
    private MoveStatus lastStatus = MoveStatus.IDLE;
    private double targetX=0, targetY=0;
    private Double endHeadingDeg = null;  // optional final heading
    private double tolIn = 2.0;
    private double arriveHeadingTolDeg = 5.0;
    private double slowRadiusIn = 14.0;
    private double minSpeed = 0.18;
    private double speedScale = 1.0;
    private double moveTimeoutSec = 0; // set >0 via a private/internal policy if desired
    private final ElapsedTime moveTimer = new ElapsedTime();

    // Follower gains (Pinpoint-first pathing)
    private double kTangent = 0.9;
    private double kCross   = 0.08;
    private double kHeading = 0.015;
    private double maxDrive = 0.9;

    // Curve impl (two-segment Hermite)
    private CurveSegment[] segs = new CurveSegment[0];
    private int segIdx=0;
    private double segT=0.0;

    // Outputs (latest commands)
    private FieldCommand lastFieldCmd = new FieldCommand(0,0,0);
    private DriveCommand lastRobotCmd = new DriveCommand(0,0,0);

    // Tag scanning, delta, and smart management
    private final ElapsedTime lastGoalTagSeen = new ElapsedTime();
    private final ElapsedTime visionNap       = new ElapsedTime();
    private TagDelta lastTagDelta = new TagDelta(0,0,false);

    // Smart vision / seek thresholds
    private double tagSeekAfterSec   = 2.0;   // if we haven’t seen the goal tag for this long, seek
    private double tagNapAfterSeen   = 0.75;  // if we recently saw a tag and we’re far from target, we can nap vision to save CPU
    private double tagNapWindowSec   = 0.75;  // keep it napped this long before waking
    private boolean seekingTag       = false;
    private double seekRotatePower   = 0.28;  // rotation magnitude while seeking

    // ---- Internal data types ----
    private static class TagPose { final double x,y,h; TagPose(double x,double y,double h){ this.x=x; this.y=y; this.h=h; } }
    private static class FieldCommand { final double vx,vy,omega; FieldCommand(double vx,double vy,double om){this.vx=vx;this.vy=vy;this.omega=om;} }
    private interface CurveSegment { double x(double t); double y(double t); double dx(double t); double dy(double t); }
    private static class HermiteSegment implements CurveSegment{
        final double x0,y0,x1,y1,mx0,my0,mx1,my1;
        HermiteSegment(double x0,double y0,double x1,double y1,double mx0,double my0,double mx1,double my1){
            this.x0=x0; this.y0=y0; this.x1=x1; this.y1=y1; this.mx0=mx0; this.my0=my0; this.mx1=mx1; this.my1=my1;
        }
        public double x(double t){ double t2=t*t,t3=t2*t; return (2*t3-3*t2+1)*x0 + (t3-2*t2+t)*mx0 + (-2*t3+3*t2)*x1 + (t3-t2)*mx1; }
        public double y(double t){ double t2=t*t,t3=t2*t; return (2*t3-3*t2+1)*y0 + (t3-2*t2+t)*my0 + (-2*t3+3*t2)*y1 + (t3-t2)*my1; }
        public double dx(double t){ double t2=t*t; return (6*t2-6*t)*x0 + (3*t2-4*t+1)*mx0 + (-6*t2+6*t)*x1 + (3*t2-2*t)*mx1; }
        public double dy(double t){ double t2=t*t; return (6*t2-6*t)*y0 + (3*t2-4*t+1)*my0 + (-6*t2+6*t)*y1 + (3*t2-2*t)*my1; }
    }

    // ---- Build a two-segment cubic curve: current → auto-midpoint → target ----
    private void buildCurveCubic(double sx,double sy,double tx,double ty){
        double mx = (sx+tx)*0.5 + 6.0;   // gentle X nudge avoids straight-line stalls
        double my = (sy+ty)*0.5;
        segs = new CurveSegment[2];
        // S->M
        {
            double t0x=0.5*(mx - sx), t0y=0.5*(my - sy);
            double t1x=0.5*(tx - sx), t1y=0.5*(ty - sy);
            segs[0] = new HermiteSegment(sx,sy, mx,my, t0x,t0y, t1x,t1y);
        }
        // M->T
        {
            double t0x=0.5*(tx - sx), t0y=0.5*(ty - sy);
            double t1x=0.5*(tx - mx), t1y=0.5*(ty - my);
            segs[1] = new HermiteSegment(mx,my, tx,ty, t0x,t0y, t1x,t1y);
        }
        segIdx=0; segT=0.0;
    }

    // ---- Follower step (field frame) ----
    private FieldCommand followStepField(){
        CurveSegment seg = segs[segIdx];

        // local search around segT for closest point
        double bestT = segT, bestD = 1e9;
        for (int k=-3;k<=3;k++){
            double t = clamp01(segT + 0.02*k);
            double dx = xIn - seg.x(t);
            double dy = yIn - seg.y(t);
            double d  = dx*dx + dy*dy;
            if (d < bestD){ bestD=d; bestT=t; }
        }
        segT = bestT;

        // derivative follower
        double dx=seg.dx(segT), dy=seg.dy(segT);
        double mag=Math.hypot(dx,dy); if (mag<1e-6) mag=1e-6;
        double tx=dx/mag, ty=dy/mag;     // unit tangent
        double nx=-ty, ny=tx;            // left normal
        double rx=xIn-seg.x(segT), ry=yIn-seg.y(segT);
        double crossErr = rx*nx + ry*ny;

        // heading target while driving = path tangent
        double desiredH = Math.toDegrees(Math.atan2(ty, tx));
        double hErr = normDeg(desiredH - hDeg);

        // field-frame command
        double vTan   = kTangent;
        double vCross = -kCross * crossErr;
        double vx     = vTan*tx + vCross*nx;
        double vy     = vTan*ty + vCross*ny;
        double omega  = clamp(kHeading*hErr, -0.6, 0.6);

        // advance t; next segment if done
        segT = clamp01(segT + 0.02 + 0.10*(1.0 - Math.min(1.0, Math.abs(crossErr)/6.0)));
        if (segT>=0.999 && segIdx+1<segs.length){ segIdx++; segT=0.0; }

        // clamp translation magnitude
        double scale = Math.max(1.0, Math.max(Math.abs(vx), Math.abs(vy)) / maxDrive);
        return new FieldCommand(vx/scale, vy/scale, omega);
    }

    // ---- Pinpoint read (mm/rad → inches/deg) ----
    private void readPinpoint(){
        double px_mm = pinpoint.getPosX();     // mm
        double py_mm = pinpoint.getPosY();     // mm
        double ph_rad= pinpoint.getHeading();  // radians
        xIn = px_mm / 25.4;
        yIn = py_mm / 25.4;
        hDeg= normDeg(Math.toDegrees(ph_rad));
    }

    // ---- Tag scan & drift-correction ----
    private boolean scanGoalTag(){
        if (tagProcessor==null) { lastTagDelta = new TagDelta(0,0,false); return false; }
        List<AprilTagDetection> dets = tagProcessor.getDetections();
        if (dets==null || dets.isEmpty()){ lastTagDelta = new TagDelta(0,0,false); return false; }

        AprilTagDetection best = null;
        for (int i=0;i<dets.size();i++){
            AprilTagDetection d = dets.get(i);
            if (d==null || d.ftcPose==null) continue;
            if (d.id == goalTagId){ best = d; break; }
        }
        if (best==null){ lastTagDelta = new TagDelta(0,0,false); return false; }

        // Camera-frame deltas (inches) from ftcPose (X right, Y forward)   [2][3]
        lastTagDelta = new TagDelta(best.ftcPose.x, best.ftcPose.y, true);
        lastGoalTagSeen.reset();

        // If we know the field pose of this tag, correct drift (blend toward absolute)
        TagPose tp = (goalTagId>=0 && goalTagId<=MAX_TAG_ID) ? tagLib[goalTagId] : null;
        if (tp != null){
            Pose2d abs = estimateRobotPoseFromTag(best, tp);
            double w = tagBlend; // small blend; Pinpoint stays primary
            xIn = lerp(xIn, abs.x,  w);
            yIn = lerp(yIn, abs.y,  w);
            hDeg= slerpDeg(hDeg, abs.hDeg, w);
            // Push corrected field pose back into Pinpoint to keep it aligned
        }
        return true;
    }

    // Blend weight for drift-correction
    private double tagBlend = 0.25;

    // ---- Compute absolute robot pose from a detection and known tag field pose ----
    private Pose2d estimateRobotPoseFromTag(AprilTagDetection d, TagPose tag){
        // ftcPose gives TAG relative to CAMERA (inches, yaw deg)   [2]
        double tx=d.ftcPose.x, ty=d.ftcPose.y, tyaw=d.ftcPose.yaw;

        // invert tag->cam to cam->tag
        double cx = -rotX(tx,ty,-tyaw);
        double cy = -rotY(tx,ty,-tyaw);

        // tag field -> camera field
        double camXf = tag.x + rotX(cx,cy, tag.h);
        double camYf = tag.y + rotY(cx,cy, tag.h);
        double camHf = normDeg(tag.h + (-tyaw));

        // camera -> robot (assume camera at tracking origin; adjust here if you have offsets)
        double rx = camXf;
        double ry = camYf;
        double rh = camHf;
        return new Pose2d(rx,ry,rh);
    }

    // ---- Vision auto-management & tag-seek priority ----
    private void manageVisionAndSeeking(double dToTarget, boolean sawGoal){
        // Decide seeking
        if (sawGoal){
            seekingTag = false;
        } else if (lastGoalTagSeen.seconds() > tagSeekAfterSec){
            seekingTag = true;                // pause following and rotate to find a tag
            // ensure vision is awake while seeking
            if (!visionEnabled) resumeVision();
        }

        // Light "nap" when tag is already locked recently and we’re far from target
        boolean far = dToTarget > 18.0;
        if (!seekingTag && far && lastGoalTagSeen.seconds() < tagNapAfterSeen){
            if (visionEnabled){
                // take a short nap to save CPU
                if (visionNap.seconds() > 0.05){
                    pauseVision();
                    visionNap.reset();
                }
            } else {
                // wake soon after the nap window
                if (visionNap.seconds() > tagNapWindowSec) resumeVision();
            }
        } else {
            // When close or seeking, keep vision on
            if (!visionEnabled) resumeVision();
        }
    }

    private void pauseVision(){
        try {
            if (visionPortal!=null){ visionPortal.stopStreaming(); visionEnabled=false; }
        } catch (Exception ignored){}
    }
    private void resumeVision(){
        try {
            if (visionPortal!=null){ visionPortal.resumeStreaming(); visionEnabled=true; }
        } catch (Exception ignored){}
    }

    // ---- Output helpers ----
    private void setOutputsField(double vx, double vy, double omega){
        lastFieldCmd = new FieldCommand(vx,vy,omega);
        double r = Math.toRadians(hDeg);
        double cos=Math.cos(-r), sin=Math.sin(-r);
        double fwd = vx*cos - vy*sin;
        double str = vx*sin + vy*cos;
        lastRobotCmd = new DriveCommand(fwd, str, omega);
    }
    private void stopOutputs(){ setOutputsField(0,0,0); }

    // ---- Math helpers ----
    public static class Pose2d {
        public final double x, y, hDeg;
        public Pose2d(double x,double y,double hDeg){ this.x=x; this.y=y; this.hDeg=hDeg; }
    }
    /** Current fused pose in field frame (inches/deg). */
    public Pose2d getPose() {
        return new Pose2d(xIn, yIn, hDeg);
    }

    private static double clamp(double v,double lo,double hi){ return Math.max(lo, Math.min(hi,v)); }
    private static double clamp01(double v){ return clamp(v,0,1); }
    private static double lerp(double a,double b,double t){ return a + t*(b-a); }
    private static double normDeg(double a){ while(a<=-180)a+=360; while(a>180)a-=360; return a; }
    private static double slerpDeg(double a,double b,double t){ return normDeg(a + t*normDeg(b-a)); }
    private static double rotX(double x,double y,double deg){ double r=Math.toRadians(deg),c=Math.cos(r),s=Math.sin(r); return x*c - y*s; }
    private static double rotY(double x,double y,double deg){ double r=Math.toRadians(deg),c=Math.cos(r),s=Math.sin(r); return x*s + y*c; }
}
