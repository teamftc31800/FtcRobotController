package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;
import org.firstinspires.ftc.teamcode.mechanisms.RPM_per_dist;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@Autonomous(name = "RedTeamBotAutoFarShortAutoadjust", group = "Examples")
public class RedTeamBotAutoFarShort_Autoadjust extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, intakeTimer;

    private ElapsedTime shootDelayTimer; // Add this at the top along with other timers
    final double SHOOT_DELAY_MILLISECONDS = 2000; // delay between left and right shots
    private DcMotor intake = null;
    private boolean hasIntake = false;
    private Servo arm = null;

    // Servo positions
    private static final double SERVO_HOME = 0.0;
    private static final double SERVO_ACTIVE = 0.1;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_STEP = 0.1; // speed per loop

    // --- Arm servo angle mapping (EDIT to match your mechanism) ---
    private static final double ARM_MIN_DEG = 0.0;    // angle at SERVO_MIN
    private static final double ARM_MAX_DEG = 300.0;  // angle at SERVO_MAX

    private enum PATHSTATE {
        PRELOAD_TO_SCORE,
        SHOOT,
        SHOOT_UPDATE,
        SCORE_TO_LEAVE,
        END,
        EXIT
    }

    private enum SHOOTSTATE {

        SHOOT_1,
        SHOOT_1_LEFT_UPDATE,

        SHOOT_1_DELAY,
        SHOOT_1_RIGHT_UPDATE,

        INTAKE,

        SHOOT_2,
        SHOOT_2_LEFT_UPDATE,
        SHOOT_2_DELAY,
        SHOOT_2_RIGHT_UPDATE,

        SHOOT_END
    }


    private SHOOTSTATE shootstate;
    private PATHSTATE pathState;

    private final Pose scorePose = new Pose(78.353, 16.706, Math.toRadians(240));
    private final Pose endPose = new Pose(99.765, 16.235, Math.toRadians(240));

    private Path leavelaunchline;

    private FeederLauncher leftFeederLauncher = new FeederLauncher();
    private FeederLauncher rightFeederLauncher = new FeederLauncher();

    // AprilTag vision — measures distance to target for RPM/angle lookup
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTagProcessor = null;
    private double webcamDistance = 0.0;
    private double webcamBearing = 0.0;
    private boolean webcamTagDetected = false;
    private static final int TARGET_TAG_ID = 24; // Red alliance tag

    // Distance-based RPM and arm angle lookup (reused from TeleOp)
    private final RPM_per_dist distToRPM = new RPM_per_dist();
    private double targetShootRPM = 3075; // fallback if no tag detected

    // RGB indicator — shows auto-adjust status
    private final RGBIndicatorLight light = new RGBIndicatorLight();

    // ------------------- PATH BUILDING -------------------
    public void buildPaths() {
        leavelaunchline = new Path(new BezierLine(scorePose, endPose));
        leavelaunchline.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());
    }

    // ------------------- PATH STATE MACHINE -------------------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_TO_SCORE:
                actionTimer.reset();
                setPathState(PATHSTATE.SHOOT_UPDATE);
                shootstate = SHOOTSTATE.SHOOT_1;
                break;

            case SHOOT_UPDATE:
                shootUpdate();
                break;

            case SCORE_TO_LEAVE:
                if (!follower.isBusy()) {
                    follower.followPath(leavelaunchline, true);
                    setPathState(PATHSTATE.END);
                }
                break;

            case END:
                if (!follower.isBusy()) {
                    setPathState(PATHSTATE.EXIT);
                }
                break;
        }
    }

    public void setPathState(PATHSTATE state) {
        pathState = state;
        pathTimer.reset();
    }

    // ------------------- SHOOTING -------------------
    public void shootLeftArtifacts() {
        leftFeederLauncher.launch(targetShootRPM, 0, true, true);
    }

    public void shootRightArtifacts() {
        rightFeederLauncher.launch(targetShootRPM, 0, true, true);
    }

    public void shootUpdate() {
        switch (shootstate) {
            case SHOOT_1:
                // Measure distance and set RPM + arm angle before shooting
                adjustShootingParams();
                shootLeftArtifacts();
                shootDelayTimer = new ElapsedTime(); // initialize timer for delay
                shootstate = SHOOTSTATE.SHOOT_1_LEFT_UPDATE;
                break;

            case SHOOT_1_LEFT_UPDATE:
                // Wait until left has launched
                if (leftFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootDelayTimer.reset(); // start counting for right shoot delay
                    shootstate = SHOOTSTATE.SHOOT_1_DELAY;
                }
                break;

            case SHOOT_1_DELAY:
                if (shootDelayTimer.milliseconds() >= SHOOT_DELAY_MILLISECONDS) {
                    shootRightArtifacts();
                    shootDelayTimer.reset();

                    shootstate = SHOOTSTATE.SHOOT_1_RIGHT_UPDATE;
                }
                break;

            case SHOOT_1_RIGHT_UPDATE:
                // Wait until delay has passed


                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.INTAKE;
                    intakeTimer.reset();
                }
                break;

            case INTAKE:
                if (intakeTimer.milliseconds() < 3000) {
                    if (hasIntake) {
                        intake.setPower(1.0);
                    }
                } else {
                    if (hasIntake) {
                        intake.setPower(0.0);
                    }
                    shootstate = SHOOTSTATE.SHOOT_2;
                }
                break;

            case SHOOT_2:
                // Re-measure distance (may have shifted after intake)
                adjustShootingParams();
                shootLeftArtifacts();
                shootDelayTimer.reset();
                shootstate = SHOOTSTATE.SHOOT_2_LEFT_UPDATE;
                break;

            case SHOOT_2_LEFT_UPDATE:
                if (leftFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootDelayTimer.reset();
                    shootstate = SHOOTSTATE.SHOOT_2_DELAY;
                }
                break;

            case SHOOT_2_DELAY:
                if (shootDelayTimer.milliseconds() >= SHOOT_DELAY_MILLISECONDS) {
                    shootRightArtifacts();
                    shootDelayTimer.reset();

                    shootstate = SHOOTSTATE.SHOOT_2_RIGHT_UPDATE;
                }

                break;
            case SHOOT_2_RIGHT_UPDATE:

                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.SHOOT_END;
                }
                break;

            case SHOOT_END:
                setPathState(PATHSTATE.SCORE_TO_LEAVE);
                break;
        }

        // Update launchers
        leftFeederLauncher.update();
        rightFeederLauncher.update();

        // Recovery timeout
        if (actionTimer.milliseconds() >= 23000) {
            setPathState(PATHSTATE.SCORE_TO_LEAVE);
            shootstate = SHOOTSTATE.SHOOT_END;
        }

        telemetry.addData("Shoot State: ", shootstate);
    }


    // ------------------- OP MODE LOOP -------------------
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("LeftFeederLauncher state", leftFeederLauncher.getStatus());
        telemetry.addData("RightFeederLauncher state", rightFeederLauncher.getStatus());
        leftFeederLauncher.reportHardwareStatus();
        rightFeederLauncher.reportHardwareStatus();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // ------------------- INIT -------------------
    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        opmodeTimer.reset();

        shootstate = SHOOTSTATE.SHOOT_1;

        leftFeederLauncher.init(hardwareMap, telemetry, "launcher", "feederServoLeft", -1);
        rightFeederLauncher.init(hardwareMap, telemetry, "launcher", "feederServoRight", 1);

        intake = hardwareMap.get(DcMotor.class, "intake");
        hasIntake = (intake != null);
        if (hasIntake) intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(Servo.class, "armServo");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(scorePose);

        telemetry.addData("intake", hasIntake ? "OK" : "MISSING");
        arm.scaleRange(0.2, 0.8);

        // RGB indicator
        light.init(hardwareMap, telemetry, "indicator");

        // AprilTag vision — same setup as SyncFeeder TeleOp
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .setCameraResolution(new android.util.Size(640, 480))
                    .build();

            // Wait for camera with 3-second timeout
            Thread.sleep(250);
            long webcamTimeout = System.currentTimeMillis() + 3000;
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                if (System.currentTimeMillis() > webcamTimeout) {
                    visionPortal.close();
                    visionPortal = null;
                    telemetry.addLine("⚠️ Webcam timed out");
                    break;
                }
                Thread.sleep(50);
            }
            if (visionPortal != null) telemetry.addLine("Webcam OK");
        } catch (Exception e) {
            telemetry.addData("⚠️ Webcam init failed", e.getMessage());
            visionPortal = null;
        }
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(PATHSTATE.PRELOAD_TO_SCORE);

        leftFeederLauncher.launch(0, 0, false, false);
        rightFeederLauncher.launch(0, 0, false, false);

        // Arm angle is now set dynamically by adjustShootingParams() before each shot
        // Set initial position as fallback
        arm.setPosition(degToServoPos(120.0));
    }

    @Override
    public void stop() {
        leftFeederLauncher.stop();
        rightFeederLauncher.stop();
        if (hasIntake) intake.setPower(0);
        if (visionPortal != null) {
            try {
                visionPortal.stopStreaming();
                visionPortal.close();
            } catch (Exception e) {}
            visionPortal = null;
        }
    }


    private double degToServoPos(double degrees) {
        // normalize 0..1 in angle space
        double t = (degrees - ARM_MIN_DEG) / (ARM_MAX_DEG - ARM_MIN_DEG);
        t = Math.max(0.0, Math.min(1.0, t));   // clamp
        // map into [SERVO_MIN, SERVO_MAX]
        return SERVO_MIN + t * (SERVO_MAX - SERVO_MIN);
    }

    // ------------------- APRIL TAG VISION -------------------
    // Scans for target tag and updates webcamDistance/webcamBearing
    private void updateAprilTag() {
        webcamTagDetected = false;
        if (aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                webcamTagDetected = true;
                webcamDistance = detection.ftcPose.range;
                webcamBearing = detection.ftcPose.bearing;
                break;
            }
        }
    }

    // Reads AprilTag distance, looks up RPM and arm angle from table, sets arm
    private void adjustShootingParams() {
        updateAprilTag();
        if (webcamTagDetected) {
            // Distance-based RPM from lookup table
            targetShootRPM = distToRPM.getFlywheelRPMForDistance(webcamDistance);

            // Distance-based arm angle from lookup table
            double targetAngle = distToRPM.getArmAngleForDistance(webcamDistance);
            arm.setPosition(degToServoPos(targetAngle));

            light.green();  // GREEN = tag detected, auto-adjust active

            telemetry.addData("AutoAdjust", "%.0f\" → %.0f RPM, %.0f°",
                    webcamDistance, targetShootRPM, targetAngle);
        } else {
            light.white();  // WHITE = no tag, using fallback
            telemetry.addLine("AutoAdjust: no tag — using fallback 3075 RPM / 120°");
            targetShootRPM = 3075;
            arm.setPosition(degToServoPos(120.0));
        }
    }
}
