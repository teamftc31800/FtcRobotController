package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.mechanisms.CameraSettings;
import org.firstinspires.ftc.teamcode.mechanisms.RPM_per_dist;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeamBotTeleop_Testing", group="Drive")
public class TeamBotTeleop_Testing extends OpMode {

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

    // Auto-orientation is active only while X button is HELD down (not toggled)

    private double webcamBearing = 0.0;         // Bearing from webcam
    private double webcamDistance = 0.0;        // Distance from webcam
    private boolean webcamTagDetected = false;  // Whether webcam sees tag

    // Auto-orientation control parameters
    private static final double ORIENTATION_KP = 0.02;           // Proportional gain for turning
    private static final double ORIENTATION_TOLERANCE = 3.0;     // Degrees - consider "aligned" within this
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
    private boolean useManualRPM = false;   // true when X-preset (3150 RPM) is active
    private String manualPresetLabel = "";     // which preset is active (RT/LT)

    private boolean lastButtonState = false;

    // Convenience: did each motor initialize?
    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;
    private boolean hasIntake     = false;

    private double largest_overshoot = 0;
    private static final double F                = 400.0;  // suggested feed speed
    private static final double FEEDER_HOLD_RPM  = 0.0;    // stop when not ready

    // Ticks per revolution (output shaft)
    private static final double FLYWHEEL_TPR = 28.0;
    private static final double FEEDER_TPR   = 537.7;

    private static final double FEEDER_RPM = 400;

    // Targets
    private static double FLYWHEEL_TARGET_RPM = 2400.;
    private static final double FLYWHEEL_TOLERANCE   = 100.0;  // Tighter tolerance to prevent firing during overshoot

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
    private static final double SERVO_STEP = 0.1; // speed per loop

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

    private boolean armAutoAim = false;       // Y button enables auto-aim
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
    private static final double AUTO_SHOOT_DURATION_MULTIPLIER = 1.4;  // Run servos 25% longer when auto-aiming
    private static final double AUTO_SHOOT_GAP_SECONDS = 0.5;  // 0.5 second gap between servos in auto-shoot mode

    // Edge detection for delay adjustment
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Synchronized feeding state
    private boolean feedingActive = false;
    private long feedingStartTime = 0;
    private boolean lastYButton = false;  // Edge detect for Y button toggle (auto-aim)
    private boolean lastAButton = false;  // Edge detect for A button toggle (feeding)

    // -----------------------------
    // RPM MANUAL ADJUSTMENT (TESTING MODE)
    // -----------------------------
    private boolean lastGp1DpadUp = false;    // Edge detect for gamepad1 D-pad up
    private boolean lastGp1DpadDown = false;  // Edge detect for gamepad1 D-pad down
    private static final double RPM_STEP = 50.0;  // Adjust by 50 RPM per click

    @Override
    public void init() {
        // Load saved camera settings (survives reboot)
        CameraSettings.load(hardwareMap);

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
        if (hasIntake)     intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hasFlywheel) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // We'll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.FORWARD);
        }

        if (hasFlywheel2) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // We'll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel2.setDirection(DcMotor.Direction.REVERSE);
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
            armServo.setPosition(SERVO_HOME);   // start position
            hasArmServo = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Arm servo not found (name: 'armServo').");
        }

        // Webcam AprilTag Detection for Auto-Orientation and Auto-Aim
        try {
            // Create AprilTag processor with optimized settings
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setNumThreads(3)
                    .build();

            // Create vision portal with Logitech C270 webcam
            VisionPortal.Builder builder = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    // 640x480 uses built-in SDK calibration data for C270 → accurate pose/distance
                    .setCameraResolution(new android.util.Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(false);

            visionPortal = builder.build();

            // Wait for camera to start
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Webcam", "Waiting for camera...");
                telemetry.update();
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }

            // Set manual exposure for consistent detection
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl != null) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(CameraSettings.exposure, TimeUnit.MILLISECONDS);
                }

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                if (gainControl != null) {
                    gainControl.setGain(CameraSettings.gain);
                }
            } catch (Exception ex) {
                // Camera controls not supported
            }

            telemetry.addLine("✅ Webcam connected - Auto-orientation ready");
        } catch (Exception e) {
            telemetry.addLine("⚠️ Webcam not found (name: 'Webcam 1')");
            visionPortal = null;
        }

        // RGB
        light.init(hardwareMap, telemetry, "indicator");
        light.blue();
    }

    @Override
    public void loop() {

        // -----------------------------
        // WEBCAM APRILTAG DETECTION (for auto-orientation AND auto-aim)
        // -----------------------------
        webcamTagDetected = false;

        if (visionPortal != null && aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Look for RED GOAL tag (ID 24) from webcam
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == RED_GOAL_TAG_ID && detection.ftcPose != null) {
                    webcamTagDetected = true;
                    webcamBearing = detection.ftcPose.bearing;  // Degrees: + = right, - = left
                    webcamDistance = detection.ftcPose.range;   // Inches
                    break;  // Only track first matching tag
                }
            }
        }

        // RGB indicator based on webcam detection
        if (webcamTagDetected && Math.abs(webcamBearing) < 5.0) {
            light.green();  // Aligned with target
        } else if (webcamTagDetected) {
            light.blue();   // Tag visible but not aligned
        } else {
            light.blue();   // No tag detected
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
            double bearingError = -webcamBearing;  // Negate so positive error = turn right

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

        // Store rotation value for telemetry debugging
        double finalRotationValue = rx;

        //----------------------------------
        // 1B. RPM MANUAL ADJUSTMENT (gamepad1 D-pad UP/DOWN) - TESTING MODE
        //----------------------------------
        boolean gp1DpadUpNow = gamepad1.dpad_up;
        boolean gp1DpadDownNow = gamepad1.dpad_down;

        // Rising edge detect: increase RPM
        if (gp1DpadUpNow && !lastGp1DpadUp) {
            FLYWHEEL_TARGET_RPM += RPM_STEP;
            useManualRPM = true;
            manualPresetLabel = String.format("Manual: %.0f RPM", FLYWHEEL_TARGET_RPM);
        }

        // Rising edge detect: decrease RPM
        if (gp1DpadDownNow && !lastGp1DpadDown) {
            FLYWHEEL_TARGET_RPM -= RPM_STEP;
            // Prevent negative RPM
            if (FLYWHEEL_TARGET_RPM < 0) {
                FLYWHEEL_TARGET_RPM = 0;
            }
            useManualRPM = true;
            manualPresetLabel = String.format("Manual: %.0f RPM", FLYWHEEL_TARGET_RPM);
        }

        lastGp1DpadUp = gp1DpadUpNow;
        lastGp1DpadDown = gp1DpadDownNow;

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
        // 3. AUTO-AIM WITH Y BUTTON
        // Y button: Auto-adjust arm angle based on webcam tag detection
        //----------------------------------
        boolean yButtonNow = gamepad2.y;

        // Rising edge detect: toggle auto-aim mode
        if (yButtonNow && !lastYButton) {
            armAutoAim = !armAutoAim;
            if (armAutoAim && webcamTagDetected && hasArmServo) {
                // Use new piecewise linear equations for both angle and RPM
                double targetAngle = distToRPM.getArmAngleForDistance(webcamDistance);
                double targetPos = degToServoPos(targetAngle);
                armServo.setPosition(targetPos);

                // Also adjust flywheel RPM based on distance
                if (hasFlywheel) {
                    FLYWHEEL_TARGET_RPM = distToRPM.getFlywheelRPMForDistance(webcamDistance);
                    useManualRPM = true;
                    String mode = distToRPM.getShootingMode(webcamDistance);
                    manualPresetLabel = String.format("Auto: %.0f\" / %.0f° / %.0f RPM (%s)",
                        webcamDistance, targetAngle, FLYWHEEL_TARGET_RPM, mode);
                }
            }
        }
        lastYButton = yButtonNow;

        // Continuously update arm position and RPM if auto-aim is active and tag is detected
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

            // Always run intake from start at max speed
            if (hasIntake) {
                intake.setPower(-1.0);
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
        // 5. MANUAL INTAKE CONTROL (gamepad2 B - OVERRIDE)
        //----------------------------------
        // Allow manual override when NOT in feeding mode
        if (!feedingActive) {
            double intakePower = 0.0;
            if (gamepad2.b) {
                intakePower = -1.0;  // Reverse intake
            }
            if (hasIntake) {
                intake.setPower(intakePower);
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
        // 7. MANUAL PRESETS using X, RT & LT (disabled during auto-aim)
        // X   -> Far shooter: 120° and 3150 RPM
        // RT  -> 150° and 2400 RPM
        // LT  -> 180° and 2200 RPM
        // ----------------------------------
        if (hasArmServo && hasFlywheel && !armAutoAim) {

            // X button: Far shooter preset
            if (gamepad2.x) {
                double posFar = degToServoPos(120.0);
                armServo.setPosition(posFar);

                FLYWHEEL_TARGET_RPM = 3150.0;
                useManualRPM = true;
                manualPresetLabel = "X: Far Shooter / 120° / 3150 RPM";
            }

            // RT preset
            if (gamepad2.right_trigger > 0.5) {
                double pos120 = degToServoPos(150.0);
                armServo.setPosition(pos120);

                FLYWHEEL_TARGET_RPM = 2400.0;
                useManualRPM = true;
                manualPresetLabel = "RT: 150° / 2400 RPM";
            }

            // LT preset
            if (gamepad2.left_trigger > 0.5) {
                double pos180 = degToServoPos(180.0);
                armServo.setPosition(pos180);

                FLYWHEEL_TARGET_RPM = 2200.0;
                useManualRPM = true;
                manualPresetLabel = "LT: 180° / 2200 RPM";
            }
        }

        //----------------------------------
        // 9. FLYWHEEL + FEEDER MOTOR SEQUENCE
        //----------------------------------
        if (hasFlywheel) {
            // Spin up flywheel to target velocity
            double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
            flywheel.setVelocity(targetTPS);
            if (hasFlywheel2){flywheel2.setVelocity(targetTPS);}

            // Read actual RPM
            double currentTPS = flywheel.getVelocity(); // ticks/sec
            double currentRPM = ticksPerSecToRPM(currentTPS, FLYWHEEL_TPR);
            boolean atSpeed = Math.abs(currentRPM - FLYWHEEL_TARGET_RPM) <= FLYWHEEL_TOLERANCE;

            double overshoot = currentRPM-FLYWHEEL_TARGET_RPM;
            if (overshoot>largest_overshoot) {
                largest_overshoot = overshoot;
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

            telemetry.addData("Flywheel", "Target %.0f RPM | Now %.0f RPM %s",
                    FLYWHEEL_TARGET_RPM, currentRPM, atSpeed ? "(near target)" : "");
            telemetry.addData("Largest Overshoot", largest_overshoot);
        }

        //----------------------------------
        // 10. TELEMETRY / DRIVER FEEDBACK
        //----------------------------------
        telemetry.addLine("== TESTING MODE ==");
        telemetry.addData("GP1 D-pad UP/DOWN", "±%.0f RPM adjustment", RPM_STEP);

        telemetry.addLine("== DRIVE & AUTO-ORIENT DEBUG ==");
        telemetry.addData("X Button", gamepad1.x ? "HELD" : "Released");
        telemetry.addData("Rotation Stick", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("Rotation Applied", "%.2f", finalRotationValue);

        telemetry.addLine("== WEBCAM ==");
        if (visionPortal != null) {
            telemetry.addData("Status", "Connected - FPS: %.0f", visionPortal.getFps());

            if (aprilTag != null) {
                List<AprilTagDetection> allDetections = aprilTag.getDetections();
                telemetry.addData("Tags Visible", allDetections.size());

                // Show all tag IDs
                if (!allDetections.isEmpty()) {
                    StringBuilder tagIds = new StringBuilder();
                    for (AprilTagDetection det : allDetections) {
                        tagIds.append(det.id).append(" ");
                    }
                    telemetry.addData("IDs", tagIds.toString());
                }
            }
        } else {
            telemetry.addData("Status", "❌ NOT CONNECTED");
        }

        if (webcamTagDetected) {
            telemetry.addData("Tag 24", "✅ %.0f\" @ %.0f°", webcamDistance, webcamBearing);
            telemetry.addData("Aligned", Math.abs(webcamBearing) <= ORIENTATION_TOLERANCE ? "✅ YES" : "NO");
        } else {
            telemetry.addData("Tag 24", "❌ Not Found");
        }

        telemetry.addLine("== AUTO FEATURES ==");
        telemetry.addData("Auto-Aim (GP2 Y)", armAutoAim ? "✅ ON" : "OFF");

        if (gamepad1.x && webcamTagDetected) {
            double turnPower = Math.abs(webcamBearing) > ORIENTATION_TOLERANCE ?
                Math.min(ORIENTATION_MAX_POWER, Math.max(ORIENTATION_MIN_POWER, Math.abs(webcamBearing * ORIENTATION_KP))) : 0.0;
            telemetry.addData("Auto-Orient (GP1 X)", "✅ ACTIVE - Power: %.2f", turnPower);
        } else if (gamepad1.x) {
            telemetry.addData("Auto-Orient (GP1 X)", "⚠️ No Tag");
        } else {
            telemetry.addData("Auto-Orient (GP1 X)", "OFF");
        }

        // Show feeding status with duration info
        if (feedingActive) {
            if (armAutoAim) {
                double extendedDelay = rightServoDelay * AUTO_SHOOT_DURATION_MULTIPLIER;
                telemetry.addData("Feeding (GP2 A)", "✅ ON - Extended (%.2fs)", extendedDelay);
            } else {
                telemetry.addData("Feeding (GP2 A)", "✅ ON - Normal (%.2fs)", rightServoDelay);
            }
        } else {
            telemetry.addData("Feeding (GP2 A)", "OFF");
        }

        // Show active preset if any
        if (useManualRPM && !manualPresetLabel.isEmpty()) {
            telemetry.addData("Preset Active", manualPresetLabel);
        }

        // Arm angle
        if (hasArmServo) {
            double deg = servoPosToDeg(armServo.getPosition());
            telemetry.addData("Arm Angle", "%.0f°", deg);
        }

        // Flywheel RPM
        if (hasFlywheel) {
            double currentRPM = ticksPerSecToRPM(flywheel.getVelocity(), 28.0);
            telemetry.addData("Flywheel", "%.0f RPM (target: %.0f)", currentRPM, FLYWHEEL_TARGET_RPM);
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
        // Default target: 2400 RPM (can be changed via presets or auto-aim)
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
        // Close the webcam vision portal when stopping
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
