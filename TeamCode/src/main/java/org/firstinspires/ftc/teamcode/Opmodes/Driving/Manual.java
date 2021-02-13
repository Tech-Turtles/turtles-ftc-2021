package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR_kotlin;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utility.Configuration.*;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.*;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double drivespeed = 1.0;
    public static double launchspeed = 0.56;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 1.0;
    public static boolean powershotMode = false;
    public static double highGoalSpeed = 0.56;
    public static double powerShotSpeed = 0.51;
    public static double wobblePower = 1.0;
    public static double manualWobblePower = 0.5;
    private WobbleStates wobbleState = WobbleStates.MANUAL;
    private boolean wobbleArrived = false;
    private final Executive.StateMachine<Manual> stateMachine;
    private TrajectoryRR_kotlin trajectoryRR;

    private Pose2d saveLocation = new Pose2d();

    private enum WobbleStates {
        MANUAL,
        UP,
        DOWN,
        STORE
    }

    public Manual() {
        stateMachine =  new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        trajectoryRR = new TrajectoryRR_kotlin(this.mecanumDrive);
        stateMachine.changeState(DRIVE, new Drive_Manual());
        stateMachine.changeState(LAUNCHER, new LaunchArm_Manual());
        stateMachine.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(lastPosition == null ? new TrajectoryRR_kotlin(mecanumDrive).getSTART_CENTER() : lastPosition);
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
        mecanumDrive.update();
        chordedControls(); // Controls for when left bumper is held down.
        displayTelemetry();
        //drivetrainControls(); // In Drive_Manual
        //intakeControls(); // In Drive_Manual
        //armControls(); // In LaunchArm_Manual
        //launcherControls(); // In LaunchArm_Manual

        /*  Exit auto operation if driver inputs are used.
        if(isDrivetrainManualInputActive() && stateMachine.getCurrentStates(DRIVE) != "Drive_Manual") {
            stopAutoDriving();
            stateMachine.changeState(DRIVE, new Drive_Manual());
            stateMachine.changeState(LAUNCHER, new LaunchArm_Manual());
        }
        */
    }

    /*
    Manual Control States
     */
    static class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            opMode.drivetrainControls();
            opMode.intakeControls();
        }
    }

    static class LaunchArm_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            opMode.armControls();
            opMode.launcherControls();
        }
    }

    /*
    End of Manual Control States
     */

    void drivetrainControls() {
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * linearSpeed * precisionMode,
                        -gamepad1.left_stick_x * lateralSpeed * precisionMode,
                        -gamepad1.right_stick_x * rotationSpeed * precisionMode
                )
        );

        if (primary.YOnce()) {
            mecanumDrive.clearEstimatedPose();
        }

        if (primary.AOnce() && !primary.start()) {
            rotationSpeed = precisionMode == 1.0 ? 0.75 : 1.0;
            precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;
        }
    }


    void intakeControls() {
        if (primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, 1f);
        } else if (primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -1f);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0f);
        }
    }

    void armControls() {
        if (secondary.XOnce())
            wobbleState = WobbleStates.STORE;
        else if (secondary.BOnce() && !secondary.start())
            wobbleState = WobbleStates.DOWN;
        else if (secondary.YOnce())
            wobbleState = WobbleStates.UP;

        switch (wobbleState) {
            case UP:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, WOBBLE_UP, wobblePower);
                break;
            case DOWN:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, WOBBLE_DOWN, wobblePower);
                break;
            case STORE:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, WOBBLE_STORE, wobblePower);
                break;
            case MANUAL:
                motorUtility.setPower(Motors.WOBBLE_ARM, -secondary.right_stick_y * manualWobblePower);
        }
        wobbleState = wobbleArrived ? WobbleStates.MANUAL : wobbleState;

        servoUtility.setPower(ContinuousServo.WOBBLE_LEFT, secondary.left_stick_y);
        servoUtility.setPower(ContinuousServo.WOBBLE_RIGHT, secondary.left_stick_y);

    }


    void launcherControls() {
        if (secondary.dpadUpOnce()) {
            launchspeed = Math.min(launchspeed + 0.01, 1.0);
        } else if (secondary.dpadDownOnce()) {
            launchspeed = Math.max(launchspeed - 0.01, 0);
        }

        if (secondary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.LAUNCHER, launchspeed);
        } else {
            motorUtility.setPower(Motors.LAUNCHER, 0f);
        }

        if (secondary.rightBumper()) {
            servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
        } else {
            servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
        }

        if (secondary.AOnce() && !secondary.start()) {
            powershotMode = !powershotMode;
            launchspeed = powershotMode ? powerShotSpeed : highGoalSpeed;
        }
    }


    void chordedControls() {
        if(primary.leftBumper()) {

            if(primary.dpadDownOnce())  // Drive to auto start position
                stateMachine.changeState(DRIVE, new Drive_ToPose(trajectoryRR.getSTART_CENTER()));
                //stateMachine.changeState(DRIVE, new Drive_ReturnToStart());  // Redundant to Drive_ToPose

            if (primary.AOnce()) // Set pose to parked center (auto start position)
                mecanumDrive.setPoseEstimate(trajectoryRR.getSTART_CENTER());

            if (primary.dpadRightOnce()) // Goto Save Location
                stateMachine.changeState(DRIVE, new Drive_ToPose(saveLocation));

            if (primary.BOnce()) // Save Location
                saveLocation = mecanumDrive.getPoseEstimate();

            if (primary.dpadUpOnce())
                stateMachine.changeState(DRIVE, new Drive_ToPose(trajectoryRR.getSHOOT_HIGHGOAL()));


            if(primary.YOnce()) { // set pose to nearest corner
                mecanumDrive.setPoseEstimate(
                        TrajectoryRR_kotlin.getNearestCornerPose2d(
                                mecanumDrive.getPoseEstimate())
                );
            }

            if(primary.dpadLeftOnce()) {
                // Do powershots
            }

            if(primary.XOnce()) {
                // Nothing defined
            }

        }
    }

    boolean isDrivetrainManualInputActive() {
        double threshold = 0.3;
        return (Math.abs(primary.left_stick_x) > threshold)
            || (Math.abs(primary.left_stick_y) > threshold)
            || (Math.abs(primary.right_stick_x) > threshold);
    }

    void displayTelemetry() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("Launcher speed:      ", df.format(launchspeed));
        telemetry.addData("Powershot mode:      ", powershotMode);
        try {
            telemetry.addData("Wobble Distance: ", df.format(getDistance(getColorSensor(ColorSensor.WOBBLE_SENSOR))));
        } catch (NullPointerException ignore) {
        }
        telemetry.addLine();
        telemetry.addLine("----Launcher----");
        telemetry.addData("Launch velocity:     ", motorUtility.getVelocity(Motors.LAUNCHER));
        telemetry.addData("Hopper position:     ", servoUtility.getAngle(Servos.HOPPER));
        telemetry.addLine();
        telemetry.addLine("----Navigation----");
        telemetry.addData("X:                   ", df.format(poseEstimate.getX()));
        telemetry.addData("Y:                   ", df.format(poseEstimate.getY()));
        telemetry.addData("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        telemetry.addData("Drive speed:         ", df.format(drivespeed));
        telemetry.addData("Precision speed:     ", df.format(precisionPercentage));
        telemetry.addData("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
    }

    void stopAutoDriving() {
        mecanumDrive.cancelFollowing();
        mecanumDrive.setDrivePower(new Pose2d());
    }


/*
Auto Control States
 */


    class Drive_ReturnToStart extends Executive.StateBase<Manual> {
        Trajectory traj_home;
        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            Pose2d startWall = new Pose2d(trajectoryRR.getSTART_CENTER().vec(),trajectoryRR.getSTART_CENTER().getHeading());
            traj_home = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate(),Math.toRadians(180.0))
                    .lineToLinearHeading(startWall)
                    .build();
            mecanumDrive.followTrajectoryAsync(traj_home);
        }

        @Override
        public void update() {
            super.update();
            if(mecanumDrive.isIdle() || isDrivetrainManualInputActive()) {
                stopAutoDriving(); // In case still mid trajectory
                nextState(DRIVE, new Drive_Manual());
            }
        }
    }



    class Drive_ToPose extends Executive.StateBase<Manual> {
        Trajectory trajectory;
        Pose2d final_pose;

        Drive_ToPose(Pose2d final_pose) {
            this.final_pose = final_pose;
        }

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            trajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(final_pose)
                    .build();
            mecanumDrive.followTrajectoryAsync(trajectory);
        }

        @Override
        public void update() {
            super.update();
            if(mecanumDrive.isIdle() || isDrivetrainManualInputActive()) {
                stopAutoDriving(); // In case still mid trajectory
                nextState(DRIVE, new Drive_Manual());
            }
        }
    }



}
