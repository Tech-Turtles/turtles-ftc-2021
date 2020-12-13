package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.BehaviorSandBox;
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
        public static double  launcherDelay = 2.0;
        public static double  servoDelay = 0.75;
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
        //ToDo: Issues with vision loading in a separate thread?
//        new Thread(() -> opmode.loadVision(true)).start();
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
            opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
        }

        @Override
        public void update() {
            super.update();

            if(opmode.ringDetector != null) {
                rings = opmode.ringDetector.getHeight();
            }

            if(stateTimer.seconds() > scanDelay)
                nextState(DRIVE, new Shoot());
        }
    }

    class Shoot extends Executive.StateBase<AutoOpmode> {
        int index = 1;

        @Override
        public void update() {
            super.update();
            if(index < 4) {
                opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
                if(stateTimer.seconds() < launcherDelay) {
                    opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                } else if(stateTimer.seconds() < launcherDelay + servoDelay) {
                    opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
                } else {
                    index++;
                    stateTimer.reset();
                }
            } else {
                opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                opmode.motorUtility.setPower(Motors.LAUNCHER, 0);
                nextState(DRIVE, new Park());
            }
        }
    }

    class Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opmode.motorUtility.setPower(Motors.FRONT_LEFT, -.5);
            opmode.motorUtility.setPower(Motors.BACK_LEFT,  -.5);
            opmode.motorUtility.setPower(Motors.FRONT_RIGHT, -.5);
            opmode.motorUtility.setPower(Motors.BACK_RIGHT, -.5);
            arrived = opmode.mecanumNavigation.getCurrentPosition().x > -16;
            if(arrived) {
                opmode.motorUtility.setPower(Motors.FRONT_LEFT, 0);
                opmode.motorUtility.setPower(Motors.BACK_LEFT, 0);
                opmode.motorUtility.setPower(Motors.FRONT_RIGHT, 0);
                opmode.motorUtility.setPower(Motors.BACK_RIGHT, 0);
                nextState(DRIVE, new Stop());
            }
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
