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
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ConfigurationVariables.*;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final RobotHardware.StartPosition startPosition;
    private Waypoints waypoints;

    @Config
    public static class AutoDashboardVariables {
        public static double  launcherDelay = 3.0;
        public static double  servoDelay = 0.4;
        public static double  scanDelay = 0.5;
        public static double launcherVelocity = 2010;

        public static double  driveSpeed = 0.7;
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
        new Thread(() -> opmode.loadVision(false)).start();
        stateMachine.changeState(DRIVE, new Start());
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
        opmode.updateMecanumHeadingFromGyroNow();
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
            opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
        }

        @Override
        public void update() {
            super.update();

            if(opMode.ringDetector != null) {
                rings = opMode.ringDetector.getHeight();
            }

            if(stateTimer.seconds() > scanDelay)
                nextState(DRIVE, new Shoot());
        }
    }

    class Shoot extends Executive.StateBase<AutoOpmode> {
        int index = 0;
        boolean doneInitialOpen = false;
        boolean finished = false;

        @Override
        public void update() {
            super.update();
            switch(index) {
                case 0:
                case 1:
                case 2:
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
                    opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                    opMode.motorUtility.setPower(Motors.LAUNCHER, 0);
                    nextState(DRIVE, new Stop());
            }
        }
    }

    class Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = driveTo(Positions.PARK.getNewNavigation2D(), driveSpeed);
            if(arrived) {
                nextState(DRIVE, new Stop());
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
