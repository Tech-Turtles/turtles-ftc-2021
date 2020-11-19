package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Waypoints;
import org.firstinspires.ftc.teamcode.Utility.Controller;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private AutoOpmode opMode;
    private Executive.StateMachine<AutoOpmode> stateMachine;
    private AllianceColor teamColor;
    private RobotHardware.StartPosition startPosition;
    private Waypoints waypoints;

    private boolean manualEnd = false;

    private final int liftRaised = 500;
    private final int liftLowered = 0;
    private final double courseTolerance = 0.5;
    private final double liftSpeed = 1.0;
    private final double grabSpeed = 0.3;
    private final double backupSpeed = 0.5;
    private final double wallStoneSpeed = 0.5;
    private final double foundationDragSpeed = 0.5;

    private final Navigation2D fudge = new Navigation2D(0,0,0);

    // Delays
    private final double start_Delay = 0.25;
    private final double scan_Delay = 0.5;
    private final double grab_Delay = 0.5;
    private final double placeFoundation_Delay = 0.25;

    private Controller controller1;


    public RobotStateContext(AutoOpmode opMode, AllianceColor teamColor, AutoOpmode.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(opMode);
        this.waypoints = new Waypoints();
        stateMachine.update();

        controller1 = opMode.primary;
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());

        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
        opMode.updateMecanumHeadingFromGyroNow();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }

    /**
     * Define Concrete State Classes
     */

    class Start_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            switch (startPosition) {
                case WALL:
                    setupInitialPosition(new Navigation2D(0, 0, 0));
                    nextState(DRIVE, new Auto_State());
                    break;
                case CENTER:
                    setupInitialPosition(new Navigation2D(0, 0, 0));
                    nextState(DRIVE, new Auto_State());
                    break;
                default:
                   throw new IndexOutOfBoundsException("Invalid start position.");
            }
        }

        private void setupInitialPosition(Navigation2D initialPosition) {
            opMode.mecanumNavigation.setCurrentPosition(initialPosition);
        }
    }

    /**
     * Loading Drive State
     * The initial start state
     */
    class Auto_State extends Executive.StateBase<AutoOpmode> {

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
        }

        @Override
        public void update() {
            super.update();
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
