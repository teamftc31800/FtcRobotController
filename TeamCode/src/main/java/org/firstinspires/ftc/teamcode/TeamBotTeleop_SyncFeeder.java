package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.mechanisms.CameraSettings;
import org.firstinspires.ftc.teamcode.mechanisms.FieldConfig;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;
import org.firstinspires.ftc.teamcode.mechanisms.RobotLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.RPM_per_dist;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeamBotTeleop_SyncFeeder", group="Drive")
public class TeamBotTeleop_SyncFeeder extends OpMode {

    // -----------------------------
    // RGB Indicator Light
    // -----------------------------
    private final RGBIndicatorLight light = new RGBIndicatorLight();
    private final RPM_per_dist distToRPM = new RPM_per_dist();

    // AprilTag tracking
    private static final int RED_GOAL_TAG_ID = 24;

    // -----------------------------
    // WEBCAM APRILTAG AUTO-ORIENTATION
    // -----------------------------
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Localizer: fused odometry + camera position
    private PinpointOdometry odometry;
    private RobotLocalizer localizer;

    // Auto-orientation is active only while X button is HELD down (not toggled)

    private double webcamBearing = 0.0;         // Bearing from webcam
    private double webcamDistance = 0.0;        // Distance from webcam
    private boolean webcamTagDetected = false;  // Whether webcam sees tag

    // Auto-orientation control parameters
    private static final double ORIENTATION_KP = 0.02;           // Proportional gain for turning
    private static final double ORIENTATION_TOLERANCE = 1.0;     // Degrees - consider "aligned" within this
    private static final double ORIENTATION_MIN_POWER = 0.1;     // Minimum turn power to overcome friction
    private static final double ORIENTATION_MAX_POWER = 0.5;     // Maximum turn power for safety

    // Drivetrain motors (312 rpm goBILDA)
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Intake (312 rpm goBILDA)
    private DcMotor intake = null;

    // Flywheel (high-speed shooter)
    private DcMotorEx flywheel = null;
    private DcMotorEx flywheel2 = null;

    // Feeder motor (312 rpm goBILDA)
    private DcMotorEx feederMotor;

    // TWO feeder servos (dual feeder)
    private CRServo feederServoLeft  = null;
    private CRServo feederServoRight = null;

    private boolean hasFlywheel = false;
    private boolean hasFlywheel2 = false;
    private boolean hasFeederMotor = false;
    private boolean hasFeederServoLeft = false;
    private boolean hasFeederServoRight = false;

    private boolean flywheelActive = false;
    private String activePreset = "";  // "" = auto-aim, "Y"/"RT"/"LT"/"Manual" = active preset

    private boolean lastButtonState = false;

    // Convenience: did each motor initialize?
    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;
    private boolean hasIntake     = false;

    // Flywheel performance stats — tracked only during auto-shoot (feedingActive)
    private double overshootSum = 0;
    private int overshootCount = 0;
    private double undershootSum = 0;
    private int undershootCount = 0;
    private boolean wasAtSpeed = false;
    private long recoveryStartTime = 0;
    private long recoverySum = 0;
    private int recoveryCount = 0;

    // Throttle flywheel getVelocity() reads — blocking USB call, run every 5th loop only
    private int loopCount = 0;
    private double cachedFlywheelRPM = 0.0;
    private static final double F                = 400.0;  // suggested feed speed
    private static final double FEEDER_HOLD_RPM  = 0.0;    // stop when not ready

    // Ticks per revolution (output shaft)
    private static final double FLYWHEEL_TPR = 28.0;
    private static final double FEEDER_TPR   = 537.7;

    private static final double FEEDER_RPM = 400;

    // Targets
    private static final double DEFAULT_FLYWHEEL_RPM = 2150.0; // Default RPM when not auto-adjusting (+150 for corrected F=13.8)
    private static double FLYWHEEL_TARGET_RPM = DEFAULT_FLYWHEEL_RPM;
    private static final double FLYWHEEL_TOLERANCE   = 100.0;  // Tighter tolerance to prevent firing during overshoot

    // PIDF tuning for flywheel velocity controller (tuned values from FlyWheelTuner.java)
    private static final double FLYWHEEL_P = 20.9;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 13.8;


    // Debounce: require N consecutive "in-tolerance" reads before feeding
    private int inToleranceCount = 0;
    private int dpad_pressed = 0;
    private boolean dpadPressed = false;

    private static final int IN_TOLERANCE_REQUIRED = 3;  // Require 3 stable readings before feeding

    private static final double targetDist = 36;
    private static final double targetDistTol = 6;

    private static final double targetBearing = -3;
    private static final double targetBearingTol = 7;

    public static double remainingDistIn = targetDist;

    // Standard servo (positional)
    private Servo armServo = null;
    private boolean hasArmServo = false;

    // Servo positions
    private static final double SERVO_HOME = 0.0;
    private static final double SERVO_ACTIVE = 0.1;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_STEP = 0.00667; // 2° per press (2 / 300° total range)

    // --- Arm servo angle mapping (EDIT to match your mechanism) ---
    private static final double ARM_MIN_DEG = 0.0;    // angle at SERVO_MIN
    private static final double ARM_MAX_DEG = 300.0;  // angle at SERVO_MAX

    // --- Arm servo bumper edge-detect (one step per press) ---
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper  = false;    

    // -----------------------------
    // ARM AUTO-AIM (distance -> angle) for RED GOAL Tag 24
    // -----------------------------
    // Distance-to-angle mapping (tune these values based on testing)
    private static final double ARM_DIST_NEAR_IN = 24.0;   // Close distance
    private static final double ARM_DIST_FAR_IN  = 60.0;   // Far distance

    private static final double ARM_DEG_NEAR = 120.0;      // Arm angle for close shots
    private static final double ARM_DEG_FAR  = 150.0;      // Arm angle for far shots

    private boolean armAutoAim = true;        // always on — RPM and arm angle auto-adjust whenever tag is visible
    private boolean lastArmToggle = false;    // edge detect for toggle

//    private final FlyWheelTuner flywheeltuner = new FlyWheelTuner();

    // Launchers
//    private final FeederLauncher leftFeederLauncher  = new FeederLauncher();
//    private final FeederLauncher rightFeederLauncher = new FeederLauncher();

    // -----------------------------
    // SYNCHRONIZED FEEDER/INTAKE CONTROL
    // -----------------------------
    private double leftServoDelay = 0.0;     // Left servo starts immediately (no delay)
    private double rightServoDelay = 1.0;    // Right servo starts after 1 second delay
    private static final double DELAY_STEP = 0.1;  // 0.1 second (100ms) per press
    private static final double MIN_DELAY = 0.0;
    private static final double MAX_DELAY = 5.0;

    // Servo speed matching
    // Left servo: 300° speed servo (standard speed)
    // Right servo: 5-turn super speed servo (needs to be slowed down to match left)
    private static final double LEFT_SERVO_POWER = -1.0;   // 300° servo at full speed
    private static final double RIGHT_SERVO_POWER = 0.5;   // Super speed servo - opposite direction, 50% power to match speed (290 RPM * 0.5 ≈ 145 RPM)

    // Auto-shoot duration multiplier (when auto-aim is active)
    private static final double AUTO_SHOOT_DURATION_MULTIPLIER = 1.4;  // Run servos 40     % longer when auto-aiming
    private static final double AUTO_SHOOT_GAP_SECONDS = 0.5;  // 0.5 second gap between servos in auto-shoot mode

    // Edge detection for delay adjustment (gamepad2)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Edge detection for RPM adjustment (gamepad1) 
    private boolean lastGp1DpadUp = false;
    private boolean lastGp1DpadDown = false;
    private static final double RPM_STEP = 50.0;

    // Synchronized feeding state
    private boolean feedingActive = false;
    private long feedingStartTime = 0;
    private boolean lastYButton = false;    // Edge detect for GP2 Y preset
    private boolean lastRTrigger = false;   // Edge detect for GP2 RT preset
    private boolean lastLTrigger = false;   // Edge detect for GP2 LT preset
    private boolean lastAButton = false;    // Edge detect for A button toggle (feeding)
    private boolean cameraRecoveryAttempted = false;  // One-shot camera recovery flag

    // Camera exposure — adjustable via GP1 bumpers (LB/RB) and D-pad L/R (gain)
    private long currentExposure = CameraSettings.exposure;
    private int  currentGain    = CameraSettings.gain;
    private static final long EXPOSURE_STEP = 1;  // ms per click
    private static final int  GAIN_STEP     = 25; // per click
    private boolean lastGp1LB = false;
    private boolean lastGp1RB = false;
    private boolean lastGp1DpadLeft  = false;
    private boolean lastGp1DpadRight = false;

    @Override
    public void init() {
        // Load saved camera settings (survives reboot)
        CameraSettings.load(hardwareMap);
        currentExposure = CameraSettings.exposure;
        currentGain = CameraSettings.gain;

        // Always reset RPM to default on each OpMode start (static variable persists between runs)
        FLYWHEEL_TARGET_RPM = DEFAULT_FLYWHEEL_RPM;

        // Motors
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");
        flywheel   = getMotorEx("launcher"); // needs DcMotorEx for velocity
        flywheel2  = getMotorEx("launcher2"); // needs DcMotorEx for velocity


//        flywheel   = flywheeltuner.init(telemetry, hardwareMap, gamepad1);

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);
        hasIntake     = (intake     != null);
        hasFlywheel   = (flywheel   != null);
        hasFlywheel2  = (flywheel2  != null);

        // Drivetrain directions
        if (hasFrontLeft)  frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft)   backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight)  backRight.setDirection(DcMotor.Direction.FORWARD);

        if (hasFrontLeft)  frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasBackLeft)  backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasFrontRight)  frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasBackRight)  backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Brake
        if (hasFrontLeft)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasIntake) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotor.Direction.REVERSE);
        }

        if (hasFlywheel) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // We'll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.FORWARD);

            // Apply tuned PIDF for faster recovery after each ball (must come after RUN_USING_ENCODER)
            PIDFCoefficients flywheelPIDF = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);
        }

        if (hasFlywheel2) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // We'll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel2.setDirection(DcMotor.Direction.REVERSE);

            // Apply tuned PIDF for faster recovery after each ball (must come after RUN_USING_ENCODER)
            PIDFCoefficients flywheel2PIDF = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel2PIDF);
        }

        // Feeder motor
        try {
            feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
            feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hasFeederMotor = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Feeder motor not found (name: 'feeder').");
        }

        // LEFT feeder servo
        try {
            feederServoLeft = hardwareMap.get(CRServo.class, "feederServoLeft");
            feederServoLeft.setPower(0.0);
            hasFeederServoLeft = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Left feeder servo not found (name: 'feederServoLeft').");
        }

        // RIGHT feeder servo
        try {
            feederServoRight = hardwareMap.get(CRServo.class, "feederServoRight");
            feederServoRight.setPower(0.0);
            hasFeederServoRight = true;
        } catch (Exception e) {
//            telemetry.addLine("⚠️ Right feeder servo not found (name: 'feederServoRight').");
        }
        // ARM SERVO
        try {
            armServo = hardwareMap.get(Servo.class, "armServo");
            armServo.scaleRange(0.2,0.8);
            armServo.setPosition(degToServoPos(120.0));  // start at 120° (close/medium shooting position)
            hasArmServo = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Arm servo not found (name: 'armServo').");
        }

        // Webcam AprilTag Detection (with retry — up to 3 attempts after crash/reboot)
        for (int camAttempt = 1; camAttempt <= 3; camAttempt++) {
            try {
                // Wait for USB stack to release — longer delay on retries to let hardware recover
                Thread.sleep(camAttempt == 1 ? 1000 : 2000);

                // Create AprilTag processor with optimized settings
                aprilTag = new AprilTagProcessor.Builder()
                        .setDrawAxes(false)           // disabled — saves CPU per frame
                        .setDrawCubeProjection(false) // disabled — most expensive draw call
                        .setDrawTagOutline(false)      // disabled — saves CPU per frame
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setNumThreads(1)              // 1 thread — frees CPU for USB interrupt handler + camera recovery
                        .build();

                // Create vision portal with Logitech C270 webcam
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .setCameraResolution(new android.util.Size(640, 480))
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .enableLiveView(false)
                        .setAutoStopLiveView(true)
                        .build();

                // Wait up to 3 seconds for camera to reach STREAMING state
                long webcamTimeout = System.currentTimeMillis() + 3000;
                while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    if (System.currentTimeMillis() > webcamTimeout) {
                        visionPortal.close();
                        visionPortal = null;
                        break;
                    }
                    telemetry.addData("Webcam", "Attempt %d/3 — Waiting...", camAttempt);
                    telemetry.update();
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException ex) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }

                // Success — apply exposure settings and stop retrying
                if (visionPortal != null) {
                    try {
                        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                        if (exposureControl != null) {
                            exposureControl.setMode(ExposureControl.Mode.Manual);
                            exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                        }
                        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                        if (gainControl != null) {
                            gainControl.setGain(currentGain);
                        }
                    } catch (Exception ex) {
                        // Camera controls not supported
                    }
                    telemetry.addLine("✅ Webcam connected (attempt " + camAttempt + ")");
                    break;  // Connected — exit retry loop
                }

                // This attempt failed — log and retry
                telemetry.addData("Webcam", "Attempt %d/3 failed — retrying...", camAttempt);
                telemetry.update();

            } catch (Exception e) {
                // Clean up failed portal before retrying
                if (visionPortal != null) {
                    try { visionPortal.close(); } catch (Exception ex) { /* ignore */ }
                    visionPortal = null;
                }
                if (camAttempt < 3) {
                    telemetry.addData("Webcam", "Attempt %d/3 error — retrying...", camAttempt);
                    telemetry.update();
                }
            }
        }

        if (visionPortal == null) {
            telemetry.addLine("⚠️ Webcam failed after 3 attempts — running without camera");
        }

        // Localizer: fused odometry + camera
        try {
            odometry = new PinpointOdometry(
                    hardwareMap,
                    0, 0, 0,  // starting pose (set per match if needed)
                    0, -2,    // pod offsets (forwardPodY=0, strafePodX=-2 from Constants.java)
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
            localizer = new RobotLocalizer(
                    odometry,
                    FieldConfig.intoTheDeep2025(),
                    0, 0, 0,  // starting field position
                    0.3       // correction alpha (0=odo only, 1=camera only)
            );
        } catch (Exception e) {
            telemetry.addLine("⚠️ Pinpoint odometry not found — localizer disabled");
        }

        // RGB
        light.init(hardwareMap, telemetry, "indicator");
        light.red();
    }

    @Override
    public void loop() {
        loopCount++;

        // -----------------------------
        // WEBCAM APRILTAG DETECTION (for auto-orientation AND auto-aim)
        // -----------------------------
        webcamTagDetected = false;

        if (visionPortal != null && aprilTag != null) {
            VisionPortal.CameraState camState = visionPortal.getCameraState();

            // One-shot recovery: if camera enters ERROR state, try resumeStreaming once
            if (camState == VisionPortal.CameraState.ERROR && !cameraRecoveryAttempted) {
                cameraRecoveryAttempted = true;
                try {
                    visionPortal.resumeStreaming();
                } catch (Exception e) {
                    // Recovery failed — camera is lost for this match
                }
            }

            // Normal detection — only when camera is actively streaming
            if (camState == VisionPortal.CameraState.STREAMING) {
                cameraRecoveryAttempted = false;  // Reset flag so future disconnects can also recover

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == RED_GOAL_TAG_ID && detection.ftcPose != null) {
                        webcamTagDetected = true;
                        webcamBearing = detection.ftcPose.bearing;
                        webcamDistance = detection.ftcPose.range;
                        break;
                    }
                }
            }
        }

        // Update localizer with all visible tags (odometry + camera fusion)
        if (localizer != null) {
            List<AprilTagDetection> allDetections = (aprilTag != null)
                    ? aprilTag.getDetections() : new ArrayList<>();
            localizer.update(allDetections);
        }

        // RGB indicator:
        //   GREEN  = tag detected, aligned (<5°)
        //   BLUE   = tag detected, not yet aligned
        //   WHITE  = no tag detected (always on)
        // When feedingActive: blinks 0.25s normal color / 0.25s red so driver knows feed is running
        if (feedingActive) {
            boolean normalPhase = (System.currentTimeMillis() % 500) < 250;
            if (normalPhase) {
                if (webcamTagDetected && Math.abs(webcamBearing) < 5.0) {
                    light.green();   // aligned
                } else if (webcamTagDetected) {
                    light.blue();    // tag visible, not aligned
                } else {
                    light.white();   // no tag — white so blink is visible even without tag
                }
            } else {
                light.red();         // blink phase — always red so driver sees feed is active
            }
        } else {
            // Not feeding — solid color based on tag state
            if (webcamTagDetected && Math.abs(webcamBearing) < 5.0) {
                light.green();       // aligned
            } else if (webcamTagDetected) {
                light.blue();        // tag visible, not aligned
            } else {
                light.white();       // no tag — stays on (white) instead of going red
            }
        }


        //----------------------------------
        // 1. DRIVE: mecanum with gamepad1 + AUTO-ORIENTATION
        //----------------------------------
        double y  = -gamepad1.left_stick_y;   // forward = +1
        double x  =  gamepad1.left_stick_x;   // strafe right = +1
        double rx =  gamepad1.right_stick_x;  // rotate right = +1

        // AUTO-ORIENTATION: Override rotation when X button is HELD and tag detected
        boolean autoOrientActive = gamepad1.x;  // Active only while button is held

        if (autoOrientActive && webcamTagDetected) {
            // Calculate correction based on bearing error
            // Bearing: Positive = tag is to the right, Negative = tag is to the left
            // At >85": aim 2° off-center to compensate for long-range arc; otherwise aim straight (0°)
            double targetBearing = (webcamDistance > 85.0) ? 2.0 : 0.0;
            double bearingError = -webcamBearing + targetBearing;

            // Only apply correction if outside tolerance
            if (Math.abs(bearingError) > ORIENTATION_TOLERANCE) {
                // Proportional control: turn to face tag
                double correctionPower = bearingError * ORIENTATION_KP;

                // Clamp to min/max power
                if (Math.abs(correctionPower) < ORIENTATION_MIN_POWER) {
                    correctionPower = Math.signum(correctionPower) * ORIENTATION_MIN_POWER;
                }
                correctionPower = Math.max(-ORIENTATION_MAX_POWER, Math.min(ORIENTATION_MAX_POWER, correctionPower));

                // Override rotation stick with auto-orientation
                rx = correctionPower;
            } else {
                // Within tolerance - stop rotating
                rx = 0.0;
            }
        }

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        flPower /= max;
        blPower /= max;
        frPower /= max;
        brPower /= max;

        if (hasFrontLeft)  frontLeft.setPower(flPower);
        if (hasBackLeft)   backLeft.setPower(blPower);
        if (hasFrontRight) frontRight.setPower(frPower);
        if (hasBackRight)  backRight.setPower(brPower);

        //----------------------------------
        // 2. DELAY ADJUSTMENT (D-PAD UP/DOWN)
        // Adjusts delay between left and right feeder servos
        //----------------------------------
        boolean dpadUpNow = gamepad2.dpad_up;
        boolean dpadDownNow = gamepad2.dpad_down;

        // Rising edge detect for UP - increase right servo delay
        if (dpadUpNow && !lastDpadUp) {
            rightServoDelay += DELAY_STEP;
            if (rightServoDelay > MAX_DELAY) {
                rightServoDelay = MAX_DELAY;
            }
        }

        // Rising edge detect for DOWN - decrease right servo delay
        if (dpadDownNow && !lastDpadDown) {
            rightServoDelay -= DELAY_STEP;
            if (rightServoDelay < MIN_DELAY) {
                rightServoDelay = MIN_DELAY;
            }
        }

        lastDpadUp = dpadUpNow;
        lastDpadDown = dpadDownNow;

        //----------------------------------
        // 2b. GAMEPAD1 D-PAD: ADJUST FLYWHEEL RPM ±50
        //----------------------------------
        boolean gp1DpadUpNow   = gamepad1.dpad_up;
        boolean gp1DpadDownNow = gamepad1.dpad_down;

        if (gp1DpadUpNow && !lastGp1DpadUp) {
            FLYWHEEL_TARGET_RPM += RPM_STEP;
            armAutoAim = false;
            activePreset = "Manual";
        }
        if (gp1DpadDownNow && !lastGp1DpadDown) {
            FLYWHEEL_TARGET_RPM -= RPM_STEP;
            if (FLYWHEEL_TARGET_RPM < 0) FLYWHEEL_TARGET_RPM = 0;
            armAutoAim = false;
            activePreset = "Manual";
        }

        lastGp1DpadUp   = gp1DpadUpNow;
        lastGp1DpadDown = gp1DpadDownNow;

        //----------------------------------
        // 2c. CAMERA EXPOSURE/GAIN — GP1 bumpers (LB/RB) + D-pad L/R
        //----------------------------------
        if (visionPortal != null) {
            ExposureControl expCtrl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainCtrl = visionPortal.getCameraControl(GainControl.class);

            // RB = increase exposure (brighter)
            boolean rbNow = gamepad1.right_bumper;
            if (rbNow && !lastGp1RB && expCtrl != null) {
                long maxExp = expCtrl.getMaxExposure(TimeUnit.MILLISECONDS);
                currentExposure = Math.min(currentExposure + EXPOSURE_STEP, maxExp);
                expCtrl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                CameraSettings.exposure = currentExposure;
                CameraSettings.save(hardwareMap);
            }
            lastGp1RB = rbNow;

            // LB = decrease exposure (darker)
            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lastGp1LB && expCtrl != null) {
                long minExp = expCtrl.getMinExposure(TimeUnit.MILLISECONDS);
                currentExposure = Math.max(currentExposure - EXPOSURE_STEP, minExp);
                expCtrl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                CameraSettings.exposure = currentExposure;
                CameraSettings.save(hardwareMap);
            }
            lastGp1LB = lbNow;

            // D-pad RIGHT = increase gain
            boolean drNow = gamepad1.dpad_right;
            if (drNow && !lastGp1DpadRight && gainCtrl != null) {
                int maxGain = gainCtrl.getMaxGain();
                currentGain = Math.min(currentGain + GAIN_STEP, maxGain);
                gainCtrl.setGain(currentGain);
                CameraSettings.gain = currentGain;
                CameraSettings.save(hardwareMap);
            }
            lastGp1DpadRight = drNow;

            // D-pad LEFT = decrease gain
            boolean dlNow = gamepad1.dpad_left;
            if (dlNow && !lastGp1DpadLeft && gainCtrl != null) {
                int minGain = gainCtrl.getMinGain();
                currentGain = Math.max(currentGain - GAIN_STEP, minGain);
                gainCtrl.setGain(currentGain);
                CameraSettings.gain = currentGain;
                CameraSettings.save(hardwareMap);
            }
            lastGp1DpadLeft = dlNow;
        }

        //----------------------------------
        // 3. AUTO-AIM — always on, updates RPM and arm angle whenever tag is visible
        //----------------------------------

        // Continuously update arm position and RPM whenever tag is detected
        if (armAutoAim && webcamTagDetected && hasArmServo) {
            // Use new piecewise linear equations for both angle and RPM
            double targetAngle = distToRPM.getArmAngleForDistance(webcamDistance);
            double targetPos = degToServoPos(targetAngle);
            armServo.setPosition(targetPos);

            // Update flywheel RPM based on detected distance
            if (hasFlywheel) {
                FLYWHEEL_TARGET_RPM = distToRPM.getFlywheelRPMForDistance(webcamDistance);
            }
        }

        //----------------------------------
        // 4. SYNCHRONIZED FEEDER/INTAKE (A BUTTON)
        // Sequence: Left servo immediately -> Right servo after delay
        //----------------------------------
        boolean aButtonNow = gamepad2.a;

        // Rising edge detect: toggle feeding state
        if (aButtonNow && !lastAButton) {
            feedingActive = !feedingActive;
            if (feedingActive) {
                feedingStartTime = System.currentTimeMillis();
                // Reset flywheel stats for this feeding session
                overshootSum = 0; overshootCount = 0;
                undershootSum = 0; undershootCount = 0;
                recoverySum = 0; recoveryCount = 0;
                wasAtSpeed = false;
            }
        }
        lastAButton = aButtonNow;

        // Execute synchronized feeding sequence
        if (feedingActive) {
            long elapsedMs = System.currentTimeMillis() - feedingStartTime;
            double elapsedSeconds = elapsedMs / 1000.0;

            // Apply duration multiplier and gap when auto-aim is active
            double effectiveServoDelay = rightServoDelay;
            double gapDuration = 0.0;  // Gap between servos (only in auto-shoot mode)

            if (armAutoAim) {
                effectiveServoDelay *= AUTO_SHOOT_DURATION_MULTIPLIER;
                gapDuration = AUTO_SHOOT_GAP_SECONDS;
            }

            // Calculate cycle time with gap
            // Cycle: left servo → gap → right servo → gap → repeat
            double cycleTime = effectiveServoDelay * 2.0 + gapDuration * 2.0;

            // Loop the sequence by taking modulo of elapsed time
            double timeInCycle = elapsedSeconds % cycleTime;

            // Run intake during shooting (hold gamepad2 X to reverse/unjam)
            if (hasIntake) {
                intake.setPower(gamepad2.x ? 1.0 : -1.0);
            }

            // EXCLUSIVE SERVO CONTROL with optional gap between servos
            if (timeInCycle < effectiveServoDelay) {
                // Phase 1: LEFT servo running
                if (hasFeederServoLeft) {
                    feederServoLeft.setPower(LEFT_SERVO_POWER);
                }
                if (hasFeederServoRight) {
                    feederServoRight.setPower(0.0);
                }

            } else if (timeInCycle < effectiveServoDelay + gapDuration) {
                // Phase 2: GAP after left servo (both servos OFF)
                if (hasFeederServoLeft) {
                    feederServoLeft.setPower(0.0);
                }
                if (hasFeederServoRight) {
                    feederServoRight.setPower(0.0);
                }

            } else if (timeInCycle < effectiveServoDelay * 2.0 + gapDuration) {
                // Phase 3: RIGHT servo running
                if (hasFeederServoLeft) {
                    feederServoLeft.setPower(0.0);
                }
                if (hasFeederServoRight) {
                    feederServoRight.setPower(RIGHT_SERVO_POWER);
                }

            } else {
                // Phase 4: GAP after right servo (both servos OFF)
                if (hasFeederServoLeft) {
                    feederServoLeft.setPower(0.0);
                }
                if (hasFeederServoRight) {
                    feederServoRight.setPower(0.0);
                }
            }
        } else {
            // Feeding stopped: turn everything off
            if (hasIntake) {
                intake.setPower(0.0);
            }
            if (hasFeederServoLeft) {
                feederServoLeft.setPower(0.0);
            }
            if (hasFeederServoRight) {
                feederServoRight.setPower(0.0);
            }
        }

        //----------------------------------
        // 5. MANUAL INTAKE CONTROL (gamepad2 B = forward, X = reverse/unjam)
        //----------------------------------
        // Allow manual override when NOT in feeding mode
        if (!feedingActive) {
            if (hasIntake) {
                if (gamepad2.x) {
                    intake.setPower(1.0);   // X held = reverse (unjam)
                } else if (gamepad2.b) {
                    intake.setPower(-1.0);  // B held = normal intake
                } else {
                    intake.setPower(0.0);
                }
            }
        }

        //----------------------------------
        // 5. MANUAL FEEDER SERVO CONTROL (D-PAD LEFT/RIGHT - OVERRIDE)
        //----------------------------------
        // Allow manual override when NOT in feeding mode
        if (!feedingActive) {
            double leftServoPower = 0.0;
            double rightServoPower = 0.0;

            if (gamepad2.dpad_left) {
                leftServoPower = -1.0;
            }
            if (gamepad2.dpad_right) {
                rightServoPower = 1.0;
            }

            if (hasFeederServoLeft) {
                feederServoLeft.setPower(leftServoPower);
            }
            if (hasFeederServoRight) {
                feederServoRight.setPower(rightServoPower);
            }
        }

        //----------------------------------
        // 6. ARM SERVO CONTROL (ONE STEP PER PRESS)
        // Only allow manual control when auto-aim is OFF
        //----------------------------------
        if (hasArmServo && !armAutoAim) {
            double servoPos = armServo.getPosition();

            boolean rbNow = gamepad2.right_bumper;
            boolean lbNow = gamepad2.left_bumper;

            // Rising-edge detect: only step once when bumper is first pressed
            if (rbNow && !lastRightBumper) {
                servoPos += SERVO_STEP;
            } else if (lbNow && !lastLeftBumper) {
                servoPos -= SERVO_STEP;
            }

            // Save states for next loop
            lastRightBumper = rbNow;
            lastLeftBumper  = lbNow;

            // Clamp and apply
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            armServo.setPosition(servoPos);
        }

        // ----------------------------------
        // 7. PRESETS — one-click toggle (Y, RT, LT)
        // Pressing active preset returns to auto-aim
        // Y   -> 120° / 3000 RPM
        // RT  -> 150° / 2550 RPM
        // LT  -> 180° / 2350 RPM
        // ----------------------------------
        if (hasArmServo && hasFlywheel) {

            // Y preset — edge-detect toggle
            boolean yNow = gamepad2.y;
            if (yNow && !lastYButton) {
                if (activePreset.equals("Y")) {
                    armAutoAim = true;
                    activePreset = "";
                } else {
                    armAutoAim = false;
                    activePreset = "Y";
                    armServo.setPosition(degToServoPos(120.0));
                    FLYWHEEL_TARGET_RPM = 3000.0;
                }
            }
            lastYButton = yNow;

            // RT preset — edge-detect on trigger threshold
            boolean rtNow = gamepad2.right_trigger > 0.5;
            if (rtNow && !lastRTrigger) {
                if (activePreset.equals("RT")) {
                    armAutoAim = true;
                    activePreset = "";
                } else {
                    armAutoAim = false;
                    activePreset = "RT";
                    armServo.setPosition(degToServoPos(150.0));
                    FLYWHEEL_TARGET_RPM = 2550.0;
                }
            }
            lastRTrigger = rtNow;

            // LT preset — edge-detect on trigger threshold
            boolean ltNow = gamepad2.left_trigger > 0.5;
            if (ltNow && !lastLTrigger) {
                if (activePreset.equals("LT")) {
                    armAutoAim = true;
                    activePreset = "";
                } else {
                    armAutoAim = false;
                    activePreset = "LT";
                    armServo.setPosition(degToServoPos(180.0));
                    FLYWHEEL_TARGET_RPM = 2350.0;
                }
            }
            lastLTrigger = ltNow;
        }

        //----------------------------------
        // 9. FLYWHEEL + FEEDER MOTOR SEQUENCE
        //----------------------------------
        if (hasFlywheel) {
            // Spin up flywheel to target velocity
            double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
            flywheel.setVelocity(targetTPS);
            if (hasFlywheel2) { flywheel2.setVelocity(targetTPS); }

            // Read actual RPM — throttled to every 5th loop to reduce blocking USB reads
            // (PIDF runs on motor controller hardware, so flywheel stays at speed between reads)
            if (loopCount % 5 == 0) {
                cachedFlywheelRPM = ticksPerSecToRPM(flywheel.getVelocity(), FLYWHEEL_TPR);
            }

            double currentRPM = cachedFlywheelRPM;
            boolean atSpeed = Math.abs(currentRPM - FLYWHEEL_TARGET_RPM) <= FLYWHEEL_TOLERANCE;

            // Track flywheel stats only during auto-shoot
            if (feedingActive && loopCount % 5 == 0) {
                double deviation = currentRPM - FLYWHEEL_TARGET_RPM;
                if (deviation > 0) {
                    overshootSum += deviation;
                    overshootCount++;
                } else if (deviation < -10) {
                    undershootSum += deviation;
                    undershootCount++;
                }
                if (wasAtSpeed && !atSpeed) {
                    recoveryStartTime = System.currentTimeMillis();
                } else if (!wasAtSpeed && atSpeed && recoveryStartTime > 0) {
                    recoverySum += System.currentTimeMillis() - recoveryStartTime;
                    recoveryCount++;
                }
                wasAtSpeed = atSpeed;
            }

            if (atSpeed) {
                inToleranceCount = Math.min(inToleranceCount + 1, IN_TOLERANCE_REQUIRED);
            } else {
                inToleranceCount = 0;
            }

            // Run feeder motor only when flywheel is stably at speed
            if (hasFeederMotor) {
                if (inToleranceCount >= IN_TOLERANCE_REQUIRED) {
                    feederMotor.setVelocity(rpmToTicksPerSec(FEEDER_RPM, FEEDER_TPR));
                }
            }

            telemetry.addData("Flywheel", "Target %.0f | Now %.0f RPM %s",
                    FLYWHEEL_TARGET_RPM, currentRPM, atSpeed ? "(at speed)" : "");
        }

        //----------------------------------
        // 10. TELEMETRY — condensed for Driver Station
        //----------------------------------
        if (visionPortal != null) {
            telemetry.addData("Cam", "%.0f FPS | Exp %dms | Gain %d", visionPortal.getFps(), currentExposure, currentGain);
        } else {
            telemetry.addData("Cam", "DOWN");
        }

        if (webcamTagDetected) {
            telemetry.addData("Tag", "%.0f\" @ %.0f°", webcamDistance, webcamBearing);
        } else {
            telemetry.addData("Tag", "No Tag");
        }

        // Auto-aim / preset state with RPM and angle
        if (hasArmServo && hasFlywheel) {
            double armDeg = servoPosToDeg(armServo.getPosition());
            if (armAutoAim) {
                telemetry.addData("Mode", "Auto | %.0f RPM | %.0f°", FLYWHEEL_TARGET_RPM, armDeg);
            } else {
                telemetry.addData("Mode", "%s | %.0f RPM | %.0f°", activePreset, FLYWHEEL_TARGET_RPM, armDeg);
            }
        }

        if (feedingActive) {
            double delay = rightServoDelay * (armAutoAim ? AUTO_SHOOT_DURATION_MULTIPLIER : 1.0);
            telemetry.addData("Feed", "ON (%.2fs)", delay);
        } else {
            telemetry.addData("Feed", "OFF");
        }

        if (feedingActive && hasFlywheel) {
            double avgOver = overshootCount > 0 ? overshootSum / overshootCount : 0;
            double avgUnder = undershootCount > 0 ? undershootSum / undershootCount : 0;
            long avgRec = recoveryCount > 0 ? recoverySum / recoveryCount : 0;
            telemetry.addData("FW Stats", "+%.0f / %.0f avg | %dms avg rec", avgOver, avgUnder, avgRec);
        }

        telemetry.addData("Intake", gamepad2.x ? "REV" : (gamepad2.b ? "RUN" : "---"));

        if (localizer != null) {
            telemetry.addData("Pos", "X=%.1f Y=%.1f H=%.0f°%s",
                    localizer.getX(), localizer.getY(), localizer.getHeadingDeg(),
                    localizer.hadTagCorrection() ? " [TAG]" : "");
        }

        telemetry.update();
    }

    // -------------------------------------------------
    // Helper: safe motor fetch (DcMotor)
    // -------------------------------------------------
    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: safe motor fetch as DcMotorEx
    // -------------------------------------------------
    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: report which motors are missing (only show if missing)
    // -------------------------------------------------
    private void reportHardwareStatus() {
        boolean anyMissing = false;
        StringBuilder missing = new StringBuilder();

        if (!hasFrontLeft) { missing.append("frontLeft "); anyMissing = true; }
        if (!hasFrontRight) { missing.append("frontRight "); anyMissing = true; }
        if (!hasBackLeft) { missing.append("backLeft "); anyMissing = true; }
        if (!hasBackRight) { missing.append("backRight "); anyMissing = true; }
        if (!hasIntake) { missing.append("intake "); anyMissing = true; }
        if (!hasFeederMotor) { missing.append("feederMotor "); anyMissing = true; }
        if (!hasFlywheel) { missing.append("flywheel "); anyMissing = true; }
        if (!hasFlywheel2) { missing.append("flywheel2 "); anyMissing = true; }
        if (!hasArmServo) { missing.append("armServo "); anyMissing = true; }
        if (!hasFeederServoLeft) { missing.append("feederServoLeft "); anyMissing = true; }
        if (!hasFeederServoRight) { missing.append("feederServoRight "); anyMissing = true; }

        if (anyMissing) {
            telemetry.addLine("== ⚠️ HARDWARE MISSING ==");
            telemetry.addData("Missing", missing.toString());
        }
    }

    // -------------------------------------------------
    // Math helpers for flywheel RPM <-> ticks/sec
    // -------------------------------------------------
    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        double ticksPerMinute = rpm * ticksPerRev;
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }
    // -------------------------------------------------
// Helper: convert servo position (0–1) to degrees
// -------------------------------------------------
    private double servoPosToDeg(double pos) {
        pos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, pos));
        double t = (pos - SERVO_MIN) / (SERVO_MAX - SERVO_MIN);
        return ARM_MIN_DEG + t * (ARM_MAX_DEG - ARM_MIN_DEG);
    }

    // -------------------------------------------------
// Helper: convert degrees to servo position (0–1)
// -------------------------------------------------
    private double degToServoPos(double degrees) {
        // normalize 0..1 in angle space
        double t = (degrees - ARM_MIN_DEG) / (ARM_MAX_DEG - ARM_MIN_DEG);
        t = Math.max(0.0, Math.min(1.0, t));   // clamp
        // map into [SERVO_MIN, SERVO_MAX]
        return SERVO_MIN + t * (SERVO_MAX - SERVO_MIN);
    }

    // -------------------------------------------------
    // Helper: Calculate arm angle based on distance to target
    // Uses linear interpolation between near and far distances
    // -------------------------------------------------
    private double calculateArmAngle(double distance) {
        // Clamp distance to valid range
        if (distance <= ARM_DIST_NEAR_IN) {
            return ARM_DEG_NEAR;
        }
        if (distance >= ARM_DIST_FAR_IN) {
            return ARM_DEG_FAR;
        }

        // Linear interpolation between near and far
        double t = (distance - ARM_DIST_NEAR_IN) / (ARM_DIST_FAR_IN - ARM_DIST_NEAR_IN);
        return ARM_DEG_NEAR + t * (ARM_DEG_FAR - ARM_DEG_NEAR);
    }

    @Override
    public void start() {
        // Start flywheels ramping up immediately when match starts
        // This is like "medium mode" - flywheel spins up as soon as you press PLAY
        // Default target: 2000 RPM (can be changed via presets or auto-aim)
        if (hasFlywheel) {
            double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
            flywheel.setVelocity(targetTPS);
        }
        if (hasFlywheel2) {
            double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
            flywheel2.setVelocity(targetTPS);
        }
    }

    @Override
    public void stop() {
        // Stop all powered devices explicitly on OpMode end

        // Flywheels — FLOAT mode means they keep spinning without this
        if (hasFlywheel)  flywheel.setVelocity(0);
        if (hasFlywheel2) flywheel2.setVelocity(0);

        // Feeder servos — CRServos hold last power without this
        if (hasFeederServoLeft)  feederServoLeft.setPower(0);
        if (hasFeederServoRight) feederServoRight.setPower(0);

        // Intake
        if (hasIntake) intake.setPower(0);

        // Fully release webcam so it reinitializes cleanly on next OpMode start
        if (visionPortal != null) {
            try {
                visionPortal.stopStreaming();   // stop stream first
                visionPortal.close();           // then release USB
            } catch (Exception e) {
                // ignore — camera may already be in a bad state
            }
            visionPortal = null;               // clear reference so GC can clean up
        }
    }
}
