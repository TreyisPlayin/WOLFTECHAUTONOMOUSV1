# package org.firstinspires.ftc.teamcode;

# 

# import android.graphics.Color;

# import com.qualcomm.robotcore.hardware.\*;

# import com.qualcomm.robotcore.util.ElapsedTime;

# import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

# 

# public class DECODEPredictiveIntake {

# 

# &nbsp;   // --- Probabilistic Parameters ---

# &nbsp;   public static double CONFIDENCE\_REQ = 0.85;     // Certainty to commit to a sort

# &nbsp;   public static double FUNNEL\_BELIEF\_GAIN = 0.25; // Predictive weight on side-funnels

# &nbsp;   public static double BELIEF\_DECAY = 0.08;       // Certainty drop rate when empty

# &nbsp;   public static double TRIGGER\_DIST\_CM = 2.4;     // Proximity threshold for V3 confirmation

# &nbsp;   public static double MISS\_TIMEOUT\_SEC = 10.0;   // Timeout for missing artifacts

# &nbsp;   

# &nbsp;   public static final char MOTIF\_MAP = {

# &nbsp;       {' ', ' ', ' '}, // IDs 0-20

# &nbsp;       {'G', 'P', 'P'}, // ID 21: GPP

# &nbsp;       {'P', 'G', 'P'}, // ID 22: PGP

# &nbsp;       {'P', 'P', 'G'}  // ID 23: PPG

# &nbsp;   };

# 

# &nbsp;   private CRServo intake, lAuger, rAuger;

# &nbsp;   private Servo lrSorter, lPusher, rPusher;

# &nbsp;   private ColorSensor funnelL, funnelR, gateFront, gateBack;

# &nbsp;   private DigitalChannel storageL, storageR; 

# &nbsp;   private VoltageSensor battery;

# 

# &nbsp;   private double pG = 0.0, pP = 0.0; // Bayesian Belief States

# &nbsp;   private int storedCount = 0;

# &nbsp;   private boolean storageExpectation = false;

# &nbsp;   private ElapsedTime ghostTimer = new ElapsedTime();

# &nbsp;   private ElapsedTime storageCycleTimer = new ElapsedTime();

# 

# &nbsp;   public DECODEPredictiveIntake(HardwareMap hw) {

# &nbsp;       intake = hw.get(CRServo.class, "intake servo");

# &nbsp;       lAuger = hw.get(CRServo.class, "leftAuger");

# &nbsp;       rAuger = hw.get(CRServo.class, "rightAuger");

# &nbsp;       lrSorter = hw.get(Servo.class, "LR pusher");

# &nbsp;       lPusher = hw.get(Servo.class, "Lpusher");

# &nbsp;       rPusher = hw.get(Servo.class, "Rpusher");

# &nbsp;       

# &nbsp;       funnelL = hw.get(ColorSensor.class, "funnelLeft");

# &nbsp;       funnelR = hw.get(ColorSensor.class, "funnelRight");

# &nbsp;       gateFront = hw.get(ColorSensor.class, "centerFront");

# &nbsp;       gateBack = hw.get(ColorSensor.class, "centerBack");

# &nbsp;       

# &nbsp;       storageL = hw.get(DigitalChannel.class, "distL");

# &nbsp;       storageR = hw.get(DigitalChannel.class, "distR");

# &nbsp;       storageL.setMode(DigitalChannel.Mode.INPUT);

# &nbsp;       storageR.setMode(DigitalChannel.Mode.INPUT);

# &nbsp;       

# &nbsp;       battery = hw.voltageSensor.iterator().next();

# &nbsp;   }

# 

# &nbsp;   /\*\*

# &nbsp;    \* Unified Asynchronous perception loop. Call this every TeleOp cycle.

# &nbsp;    \*/

# &nbsp;   public void update(double drivePower, int aprilTagId, boolean follow) {

# &nbsp;       double vScale = 12.0 / battery.getVoltage();

# &nbsp;       intake.setPower(drivePower > 0.05? 1.0 \* vScale : 0);

# 

# &nbsp;       // ASYNCHRONOUS PERCEPTION: Update beliefs regardless of motor state

# &nbsp;       updateBelief(funnelL, FUNNEL\_BELIEF\_GAIN);

# &nbsp;       updateBelief(funnelR, FUNNEL\_BELIEF\_GAIN);

# 

# &nbsp;       if (!follow) {

# &nbsp;           lrSorter.setPosition(0.5); // BYPASS: Center sorter for maximum pass-through speed

# &nbsp;           resetBelief();

# &nbsp;           return; 

# &nbsp;       }

# 

# &nbsp;       char target = (aprilTagId >= 21 \&\& aprilTagId <= 23 \&\& storedCount < 3)? MOTIF\_MAP\[storedCount] : 'N';

# 

# &nbsp;       // Decision Gate: Proximity trigger and Bayesian execution \[4]

# &nbsp;       if (((DistanceSensor) gateFront).getDistance(DistanceUnit.CM) < TRIGGER\_DIST\_CM) {

# &nbsp;           if ((target == 'G' \&\& pG > CONFIDENCE\_REQ) |

# 

# | (target == 'P' \&\& pP > CONFIDENCE\_REQ)) {

# &nbsp;               lrSorter.setPosition(storedCount % 2 == 0? 0.2 : 0.8);

# &nbsp;               if (!storageExpectation) { ghostTimer.reset(); storageExpectation = true; }

# &nbsp;           } else {

# &nbsp;               lrSorter.setPosition(0.5); // Passive Rejection

# &nbsp;           }

# &nbsp;       } else {

# &nbsp;           // Decay beliefs when empty and pre-position to neutral \[5, 6]

# &nbsp;           pG = Math.max(0, pG - BELIEF\_DECAY);

# &nbsp;           pP = Math.max(0, pP - BELIEF\_DECAY);

# &nbsp;           lrSorter.setPosition(0.5);

# &nbsp;           if (getHSV(gateBack)!= 'N') resetBelief(); // Confirmation via back sensor

# &nbsp;       }

# 

# &nbsp;       // Digital Storage logic (active-low)

# &nbsp;       manageChannel(storageL, lAuger, lPusher, vScale, true);

# &nbsp;       manageChannel(storageR, rAuger, rPusher, vScale, false);

# 

# &nbsp;       if (storageExpectation \&\& ghostTimer.seconds() > MISS\_TIMEOUT\_SEC) {

# &nbsp;           storageExpectation = false; // Self-healing miss recovery

# &nbsp;       }

# &nbsp;   }

# 

# &nbsp;   private void updateBelief(ColorSensor s, double gain) {

# &nbsp;       char color = getHSV(s);

# &nbsp;       if (color == 'G') { pG = Math.min(1.0, pG + gain); pP = Math.max(0, pP - gain); }

# &nbsp;       if (color == 'P') { pP = Math.min(1.0, pP + gain); pG = Math.max(0, pG - gain); }

# &nbsp;   }

# 

# &nbsp;   private void manageChannel(DigitalChannel ds, CRServo auger, Servo pusher, double scale, boolean left) {

# &nbsp;       if (!ds.getState() \&\& storageCycleTimer.seconds() > 0.6) {

# &nbsp;           storageCycleTimer.reset();

# &nbsp;           storageExpectation = false;

# &nbsp;           storedCount++;

# &nbsp;       }

# &nbsp;   }

# 

# &nbsp;   private char getHSV(ColorSensor s) {

# &nbsp;       float hsv = new float\[7];

# &nbsp;       Color.RGBToHSV(s.red()\*8, s.green()\*8, s.blue()\*8, hsv);

# &nbsp;       if (hsv\[8] > 0.42 \&\& hsv\[9] > 0.15) {

# &nbsp;           if (hsv >= 150 \&\& hsv <= 163) return 'G';

# &nbsp;           if (hsv >= 165 \&\& hsv <= 240) return 'P';

# &nbsp;       }

# &nbsp;       return 'N';

# &nbsp;   }

# 

# &nbsp;   private void resetBelief() { pG = 0; pP = 0; }

# }

