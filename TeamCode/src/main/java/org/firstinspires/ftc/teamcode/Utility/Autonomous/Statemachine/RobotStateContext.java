package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Positions;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Waypoints;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext.AutoDashboardVariables.*;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final RobotHardware.StartPosition startPosition;
    private Waypoints waypoints;

    @Config
    static class AutoDashboardVariables {
        public static double  hopperOpen = 0.32;
        public static double  hopperPush = 0.23;

        public static double  launcherDelay = 0.4;
        public static double  scanDelay = 0.5;

        public static double  driveSpeed = 1.0;
        public static double  launcherSpeed = 0.6;
    }

    private RingDetectionAmount rings = RingDetectionAmount.ZERO;

    public RobotStateContext(AutoOpmode opmode, AllianceColor allianceColor, AutoOpmode.StartPosition startPosition) {
        this.opmode = opmode;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(opmode);
        this.waypoints = new Waypoints(allianceColor);
        stateMachine.update();
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start());
        new Thread(() -> opmode.loadVision(true)).start();
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
        opmode.updateMecanumHeadingFromGyroNow();
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
                    setupInitialPosition(Positions.START_WALL.getNav2D());
                    nextState(DRIVE, new Initial());
                    break;
                case CENTER:
                    setupInitialPosition(Positions.START_CENTER.getNav2D());
                    nextState(DRIVE, new Initial());
                    break;
                default:
                   throw new IndexOutOfBoundsException("Invalid start position.");
            }
        }

        private void setupInitialPosition(Navigation2D initialPosition) {
            opMode.mecanumNavigation.setCurrentPosition(initialPosition);
            opMode.imuUtil.updateNow();
            opMode.imuUtil.setCompensatedHeading(Math.toDegrees(initialPosition.theta));
        }
    }

    class Initial extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.servoUtility.setAngle(Servos.HOPPER, hopperOpen);
        }

        @Override
        public void update() {
            super.update();

            if(opMode.ringDetector != null) {
                rings = opmode.ringDetector.getHeight();
            }

            if(stateTimer.seconds() > scanDelay)
                nextState(DRIVE, new PrepShoot_A());
        }
    }

    class PrepShoot_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            driveTo(Positions.SHOOT.getNewNavigation2D(), driveSpeed);
            if(arrived)
                nextState(DRIVE, new Shoot_A());
        }
    }

    class Shoot_A extends Executive.StateBase<AutoOpmode> {
        int index = 1;

        @Override
        public void update() {
            super.update();
            if(index < 4) {
                if(stateTimer.seconds() > (launcherDelay * index)) {
                    opMode.servoUtility.setAngle(Servos.HOPPER, hopperPush);
                    opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
                    index++;
                }
                opMode.servoUtility.setAngle(Servos.HOPPER, hopperOpen);
                opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
            } else {
                opMode.servoUtility.setAngle(Servos.HOPPER, hopperOpen);
                if(rings.equals(RingDetectionAmount.ZERO))
                    nextState(DRIVE, new Park());
                else
                    nextState(DRIVE, new ToRings());
            }
        }
    }

    class ToRings extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            driveTo(Positions.RINGS.getNewNavigation2D().addAndReturn(0,-10,0), driveSpeed);
            if(arrived)
                nextState(DRIVE, new GrabRings());
        }
    }

    class GrabRings extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            //todo change from hardcoded speed
            opmode.motorUtility.setPower(Motors.INTAKE, 1.0);
            driveTo(Positions.RINGS.getNewNavigation2D().addAndReturn(0,6,0), driveSpeed);
            if(arrived)
                nextState(DRIVE, new PrepShoot_B());
        }
    }

    class PrepShoot_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            driveTo(Positions.SHOOT.getNewNavigation2D(), driveSpeed);
            if(arrived)
                nextState(DRIVE, new Shoot_B());
        }
    }

    class Shoot_B extends Executive.StateBase<AutoOpmode> {
        int index = 1;

        @Override
        public void update() {
            super.update();
            if(index < 4) {
                if(stateTimer.seconds() > (launcherDelay * index)) {
                    opMode.servoUtility.setAngle(Servos.HOPPER, hopperPush);
                    opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
                    index++;
                }
                opMode.servoUtility.setAngle(Servos.HOPPER, hopperOpen);
                opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
            } else {
                nextState(DRIVE, new Park());
            }
        }
    }

    class Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            driveTo(Positions.PARK.getNewNavigation2D(), driveSpeed);
            if(arrived)
                nextState(DRIVE, new Stop());
        }
    }

    class Stop extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opmode.stop();
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
