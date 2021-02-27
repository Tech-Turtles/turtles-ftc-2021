package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR_kotlin;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext.servoDelay;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.*;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.*;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double drivespeed = 1.0;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 1.0;
    public static boolean powershotMode = false;
    public static double manualWobblePower = 0.5;

//    private DistanceSensor leftRange, backRange;

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
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
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
        trajectoryRR = new TrajectoryRR_kotlin(this.mecanumDrive);
//
//        leftRange = hardwareMap.get(DistanceSensor.class, "leftDistance");
//        backRange = hardwareMap.get(DistanceSensor.class, "backDistance");
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
        mecanumDrive.update(packet);
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
            opMode.drivetrainStandardControls();
            opMode.drivetrainUtilityControls();
            opMode.intakeControls();
            if(opMode.primary.leftBumper() && opMode.primary.startOnce())
                nextState(DRIVE, new Drive_ManualFieldCentric());
        }
    }

    static class Drive_ManualFieldCentric extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            opMode.drivetrainFieldCentricControls();
            opMode.drivetrainUtilityControls();
            opMode.intakeControls();
            if(opMode.primary.leftBumper() && opMode.primary.startOnce())
                nextState(DRIVE, new Drive_Manual());
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

    void drivetrainStandardControls() {
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * linearSpeed * precisionMode,
                        -gamepad1.left_stick_x * lateralSpeed * precisionMode,
                        -gamepad1.right_stick_x * rotationSpeed * precisionMode
                )
        );
    }

    void drivetrainFieldCentricControls() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * linearSpeed * precisionMode,
                -gamepad1.left_stick_x * lateralSpeed * precisionMode)
                .rotated(-poseEstimate.getHeading());

        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * rotationSpeed * precisionMode
                )
        );
    }

    void drivetrainUtilityControls() {
        if(!primary.leftBumper()) {
            if (primary.YOnce()) {
                mecanumDrive.clearEstimatedPose();
            }

            if (primary.AOnce() && !primary.start()) {
                rotationSpeed = precisionMode == 1.0 ? 0.75 : 1.0;
                precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;
            }
        }
    }

    boolean autoRunForward = false;
    void intakeControls() {
        if (primary.rightTriggerOnce()) {
            autoRunForward = true;
        }

        if(primary.left_trigger > deadzone) {
            autoRunForward = false;
            motorUtility.setPower(Motors.INTAKE, -intakePower);
        } else {
            motorUtility.setPower(Motors.INTAKE, autoRunForward ? intakePower : 0);
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
            highGoalSpeed = Math.min(highGoalSpeed + 0.01, 1.0);
        } else if (secondary.dpadDownOnce()) {
            highGoalSpeed = Math.max(highGoalSpeed - 0.01, 0);
        }

        if (secondary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.LAUNCHER, highGoalSpeed);
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
            highGoalSpeed = powershotMode ? powerShotSpeed : highGoalSpeed;
        }
    }


    void chordedControls() {
        if(primary.leftBumper()) {

            if(primary.dpadDownOnce())  // Drive to auto start position
                stateMachine.changeState(DRIVE, new Drive_ToPose(trajectoryRR.getSTART_CENTER()));

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
                // Hopefully no problem is created from constructing all state objects
                // before using them.
                stateMachine.changeState(DRIVE,
                        new Drive_moveAndShoot(trajectoryRR.getPOWERSHOT_RIGHT(),
                                new Drive_moveAndShoot(trajectoryRR.getPOWERSHOT_CENTER(),
                                        new Drive_moveAndShoot(trajectoryRR.getPOWERSHOT_LEFT(),
                                                new Drive_Manual_AllStates()))));
            }

            if(primary.XOnce()) {
                // Drive test to show angle error in start position
                stateMachine.changeState(DRIVE,
                        new Drive_ToPose(trajectoryRR.getSTART_CENTER()
                        .plus(new Pose2d(96,0,0))));
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
        telemetry.addData("Launcher speed:      ", df.format(highGoalSpeed));
        telemetry.addData("Powershot mode:      ", powershotMode);
        telemetry.addData("X:                   ", df.format(poseEstimate.getX()));
        telemetry.addData("Y:                   ", df.format(poseEstimate.getY()));
        telemetry.addData("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if(packet != null) {
            packet.addLine("--MANUAL--");
            packet.put("Precision mode:      ", df.format(precisionMode));
            packet.put("Launcher speed:      ", df.format(highGoalSpeed));
            packet.put("Powershot mode:      ", powershotMode);
            packet.put("Launch velocity:     ", motorUtility.getVelocity(Motors.LAUNCHER));
            packet.put("Hopper position:     ", servoUtility.getAngle(Servos.HOPPER));
            packet.put("X:                   ", df.format(poseEstimate.getX()));
            packet.put("Y:                   ", df.format(poseEstimate.getY()));
            packet.put("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
//            packet.put("leftDistance range   ", df_precise.format(leftRange.getDistance(DistanceUnit.INCH)));
//            packet.put("backDistance range   ", df_precise.format(backRange.getDistance(DistanceUnit.INCH)));
            try {
                packet.put("Wobble Distance: ", df.format(getDistance(getColorSensor(ColorSensor.WOBBLE_SENSOR))));
            } catch (NullPointerException ignore) {}
            packet.put("Drive speed:         ", df.format(drivespeed));
            packet.put("Precision speed:     ", df.format(precisionPercentage));
            packet.put("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        }
    }

    void stopAutoDriving() {
        mecanumDrive.cancelFollowing();
        mecanumDrive.setDrivePower(new Pose2d());
    }

    /*
    Auto Control States
     */
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

    class Drive_moveAndShoot extends Executive.StateBase<Manual> {
        boolean arrived = false;
        Trajectory trajectory;
        Pose2d final_pose;
        Executive.StateBase<Manual> nextDriveState;

        Drive_moveAndShoot(Pose2d final_pose, Executive.StateBase<Manual> nextDriveState) {
            this.final_pose = final_pose;
            this.nextDriveState = nextDriveState;
        }

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            trajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(final_pose)
                    .build();
            mecanumDrive.followTrajectoryAsync(trajectory);
            nextState(LAUNCHER, new Launch_windUp(powerShotSpeed));
        }

        @Override
        public void update() {
            super.update();

            if(isDrivetrainManualInputActive()) { // Return manual control
                stopAutoDriving();
                stateMachine.changeState(DRIVE, new Drive_Manual());
                stateMachine.changeState(LAUNCHER, new LaunchArm_Manual());
            }
            if(opMode.mecanumDrive.isIdle() && !arrived) {
                arrived = true;
                nextState(LAUNCHER, new Launch_fire(powerShotSpeed));
            }
            if(arrived && stateMachine.getStateReference(LAUNCHER).isDone) {
                nextState(DRIVE, nextDriveState);
            }
        }
    }

    // Restores all states to driver controlled
    static class Drive_Manual_AllStates extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            nextState(LAUNCHER, new LaunchArm_Manual());
            nextState(DRIVE, new Drive_Manual());
        }
    }

    /*
     *    launchSpeed (percent) is an argument of the
     *    constructor, so this mode works with powershots and high goal shots.
     *    However, the feedback that shows we're ready to shoot is to support moving powershots.
     */
    static class Launch_windUp extends Executive.StateBase<Manual> {
        double launchSpeed;
        double launchVelocity_tps; // encoder ticks per second

        Launch_windUp(double launchSpeed) {
            this.launchSpeed = launchSpeed;
            this.launchVelocity_tps = (0.95 * Configuration.getLaunchTicksPerSecondFromPowerSpeed(launchSpeed));
        }

        @Override
        public void update() {
            super.update();
            // Set launch speed and servo position.
            // If velocity is high enough, and servoDelay elapsed, then isDone = true
            opMode.motorUtility.setPower(Motors.LAUNCHER, this.launchSpeed);
            opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
            isDone = opMode.motorUtility.getVelocity(Motors.LAUNCHER) > this.launchVelocity_tps && stateTimer.seconds() > servoDelay;
        }
    }

    /*
     * Launch_fire
     */
    class Launch_fire extends Executive.StateBase<Manual> {
        double launchSpeed;
        double launchVelocity_tps; // encoder ticks per second
        boolean servoPushed = false;
        double manualServoDelay = 0.4;
        ElapsedTime servoTimer = new ElapsedTime();

        Launch_fire(double launchSpeed) {
            this.launchSpeed = launchSpeed;
            this.launchVelocity_tps = (0.95 * Configuration.getLaunchTicksPerSecondFromPowerSpeed(launchSpeed));
        }

        @Override
        public void update() {
            super.update();
            opMode.motorUtility.setPower(Motors.LAUNCHER, this.launchSpeed);
            if (!(servoPushed)) {
                if (opMode.motorUtility.getVelocity(Motors.LAUNCHER) > this.launchVelocity_tps) {
                    opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
                    servoPushed = true;
                    servoTimer.reset();
                }
            } else {
                if (servoTimer.seconds() > manualServoDelay) {
                    //isDone = true; // This indicates we've shot, but not that we're ready to shoot.
                    nextState(LAUNCHER, new Launch_windUp(this.launchSpeed));
                }
            }
        }
    }
}
