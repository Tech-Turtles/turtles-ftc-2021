package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Positions;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR_kotlin;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Waypoints;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.LAUNCHER;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.HOPPER_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.HOPPER_PUSH_POS;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private final Waypoints waypoints;
    private TrajectoryRR_kotlin trajectoryRR;

    public static boolean pickupRings = true;

    public static double servoDelay = 0.35;
    public static double scanDelay = 2.0;
    public static double intakeDelay = 2.3;

    public static double launcherVelocity = 1930;
    public static double launcherSpeed = 0.58;

    private RingDetectionAmount rings = RingDetectionAmount.ZERO;

    public RobotStateContext(AutoOpmode opmode, AllianceColor allianceColor, StartPosition startPosition) {
        this.opmode = opmode;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(opmode);
        this.waypoints = new Waypoints(allianceColor);
        stateMachine.update();
    }

    public void init() {
        new Thread(() -> opmode.loadVision(false)).start();
        trajectoryRR = new TrajectoryRR_kotlin(opmode.mecanumDrive);
        stateMachine.changeState(DRIVE, new Start());
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
        opmode.telemetry.addData("Rings: ", rings.name());
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }

    /**
     * Define Concrete State Classes
     */

    class Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            switch (startPosition) {
                case WALL:
                    setupInitialPosition(trajectoryRR.getSTART_WALL());
                    nextState(DRIVE, new Initial());
                    break;
                case CENTER:
                    setupInitialPosition(trajectoryRR.getSTART_CENTER());
                    nextState(DRIVE, new Initial());
                    break;
                default:
                   throw new IndexOutOfBoundsException("Invalid start position.");
            }
        }

        private void setupInitialPosition(Pose2d initialPosition) {
            opMode.imuUtil.updateNow();
            opMode.imuUtil.setCompensatedHeading(Math.toDegrees(initialPosition.getHeading()));
            opmode.mecanumDrive.setPoseEstimate(initialPosition);
        }
    }

    class Initial extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
        }

        @Override
        public void update() {
            super.update();

            nextState(DRIVE, new Scan());
        }
    }

    class Scan extends Executive.StateBase<AutoOpmode> {

        @Override
        public void update() {
            super.update();

            if(opMode.ringDetector != null)
                rings = opMode.ringDetector.getHeight();

            if(stateTimer.seconds() > scanDelay || !(rings.equals(RingDetectionAmount.ZERO))) {
                trajectoryRR.setZone(rings);
                nextState(DRIVE, new ToShoot());
            }
        }
    }

    class ToShoot extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opmode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajToShoot1());
        }

        @Override
        public void update() {
            super.update();
            opMode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
            if(opmode.mecanumDrive.isIdle()) {
                nextState(DRIVE, new FireListener());
                nextState(LAUNCHER, new Fire());
            }
        }
    }

    class PickupRings extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opmode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajPickupRings());
            // Set it to false so it won't attempt to pick up rings again
            pickupRings = false;
        }

        @Override
        public void update() {
            super.update();
            opmode.motorUtility.setPower(Motors.INTAKE, 1f);
            opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
            if(opmode.mecanumDrive.isIdle() && stateTimer.seconds() > intakeDelay) {
                nextState(DRIVE, new FromRingsShoot());
                opmode.motorUtility.setPower(Motors.INTAKE, 0);
            }
        }
    }

    class FromRingsShoot extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opmode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajToShoot2());
        }

        @Override
        public void update() {
            super.update();
            if(opmode.mecanumDrive.isIdle()) {
                nextState(DRIVE, new FireListener());
                nextState(LAUNCHER, new Fire(3));
            }
        }
    }

    class FireListener extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(stateMachine.getStateReference(LAUNCHER).isDone) {
                if(pickupRings && rings.equals(RingDetectionAmount.ONE))
                    nextState(DRIVE, new PickupRings());
                else
                    nextState(DRIVE, new Park());
                stateMachine.removeStateType(LAUNCHER);
            }
        }
    }

    class Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opmode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajToPark());
        }

        @Override
        public void update() {
            super.update();
            if(opmode.mecanumDrive.isIdle()) {
                nextState(DRIVE, new Stop());
            }
        }
    }

    class Fire extends Executive.StateBase<AutoOpmode> {
        int index;
        boolean doneInitialOpen = false;
        boolean finished = false;

        public Fire() {
            this(1);
        }

        public Fire(int startIndex) {
            this.index = startIndex;
        }

        @Override
        public void update() {
            super.update();
            switch(index) {
                case 0: index++;
                    break;
                case 1:
                case 2:
                case 3:
                    opMode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
                    if(opMode.motorUtility.getVelocity(Motors.LAUNCHER) > launcherVelocity && !doneInitialOpen) {
                        opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                        doneInitialOpen = true;
                        stateTimer.reset();
                    } else if(opMode.motorUtility.getVelocity(Motors.LAUNCHER) > launcherVelocity && stateTimer.seconds() > servoDelay) {
                        opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
                        finished = true;
                        stateTimer.reset();
                    } else if(finished && stateTimer.seconds() > servoDelay) {
                        index++;
                        doneInitialOpen = false;
                        finished = false;
                        stateTimer.reset();
                    }
                    break;
                default:
                    opMode.motorUtility.setPower(Motors.LAUNCHER, 0);
                    opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                    isDone = true;
            }
        }
    }

    class Stop extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.stop();
        }
    }

    public double getDriveScale(double seconds) {
        double driveScale;
        double speedDivider = 0.75;
        driveScale = seconds / speedDivider;
        driveScale = Math.min(driveScale, 1.0);
        return driveScale;
    }
}