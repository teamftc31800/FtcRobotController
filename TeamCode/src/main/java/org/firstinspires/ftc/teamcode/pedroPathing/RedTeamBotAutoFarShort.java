package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;

@Autonomous(name = "RedTeamBotAutoFarShort", group = "Examples")
public class RedTeamBotAutoFarShort extends OpMode {

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
        leftFeederLauncher.launch(3075, 0, true, true);
    }

    public void shootRightArtifacts() {
        rightFeederLauncher.launch(3075, 0, true, true);
    }

    public void shootUpdate() {
        switch (shootstate) {
            case SHOOT_1:
                // Start left shoot first
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
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(PATHSTATE.PRELOAD_TO_SCORE);

        leftFeederLauncher.launch(0, 0, false, false);
        rightFeederLauncher.launch(0, 0, false, false);

       // arm.setPosition(0.5);

        double presetPos = degToServoPos(120.0);
        arm.setPosition(presetPos);
    }

    @Override
    public void stop() {
        leftFeederLauncher.stop();
        rightFeederLauncher.stop();
    }


    private double degToServoPos(double degrees) {
        // normalize 0..1 in angle space
        double t = (degrees - ARM_MIN_DEG) / (ARM_MAX_DEG - ARM_MIN_DEG);
        t = Math.max(0.0, Math.min(1.0, t));   // clamp
        // map into [SERVO_MIN, SERVO_MAX]
        return SERVO_MIN + t * (SERVO_MAX - SERVO_MIN);
    }
}
