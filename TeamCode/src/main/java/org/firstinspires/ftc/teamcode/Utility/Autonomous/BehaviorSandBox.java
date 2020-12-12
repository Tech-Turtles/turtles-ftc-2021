package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utility.Controller;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.BehaviorSandBox.MotorTesterVariables.*;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext.AutoDashboardVariables.launcherSpeed;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ConfigurationVariables.HOPPER_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ConfigurationVariables.HOPPER_PUSH_POS;

public class BehaviorSandBox implements Executive.RobotStateMachineContextInterface {

    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor teamColor;
    private final RobotHardware.StartPosition startPosition;
    private Waypoints waypoints;
    private double driveSpeed = 0.8;
    private Controller controller1;

    @Config
    static class MotorTesterVariables {
        public static double forwardSpeed = 0;
        public static double rotationalSpeed = 0;
        public static double strafeSpeed = 0;

        public static double launchDelay = 2.0;
        public static double servoDelay = 1.0;
    }

    public BehaviorSandBox(AutoOpmode opmode, AllianceColor teamColor, RobotHardware.StartPosition startPosition) {
        this.opmode = opmode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opmode);
        waypoints = new Waypoints(teamColor);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_Menu());
        stateMachine.init();
        controller1 = opmode.primary;
    }

    public void update() {
        stateMachine.update();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }

    /**
     * Define Concrete State Classes
     */

    class Start_Menu extends Executive.StateBase {
        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opmode.telemetry.clear();
        }

        @Override
        public void update() {
            super.update();
            opmode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            opmode.imuUtil.updateNow();
            opmode.telemetry.addLine("---Start Menu---");
             opmode.telemetry.addLine("Manual: A");
             opmode.telemetry.addLine("Motor Tester: B");
             opmode.telemetry.addLine("Servo Tester: X");
            opmode.telemetry.addLine("Shooter Test: Y");
            if(controller1.AOnce()) stateMachine.changeState(DRIVE, new Manual());
            else if (controller1.BOnce()) stateMachine.changeState(DRIVE, new Motor_Tester());
            else if (controller1.XOnce()) stateMachine.changeState(DRIVE, new Servo_Tester());
            else if (controller1.YOnce()) stateMachine.changeState(DRIVE, new Shooter());
        }
    }

    class Manual extends Executive.StateBase {
        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opmode.telemetry.clear();
        }

        @Override
        public void update() {
            super.update();
            if(controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            }

            opmode.motorUtility.setDriveForSimpleMecanum(controller1.left_stick_x * driveSpeed, controller1.left_stick_y * driveSpeed,
                    controller1.right_stick_x * driveSpeed, controller1.right_stick_y * driveSpeed);


            for (MotorTypes type : MotorTypes.values()) {
                opmode.telemetry.addLine(type.name());
                for (Motors motor : Motors.values()) {
                    if(type != motor.getType()) continue;
                    opmode.telemetry.addData(motor.name() + ": ", opmode.motorUtility.getEncoderValue(motor));
                }
            }
        }
    }

    class Motor_Tester extends Executive.StateBase {

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opmode.telemetry.clear();
        }

        @Override
        public void update() {
            super.update();
            if(controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            } else if(controller1.AOnce()) {
                forwardSpeed = 0;
                rotationalSpeed = 0;
                strafeSpeed = 0;
            }

            if(controller1.dpadUpOnce()) forwardSpeed = Math.min(forwardSpeed + 0.01, 1.0);
            else if(controller1.dpadDown()) forwardSpeed = Math.max(forwardSpeed - 0.01, -1.0);

            if(controller1.dpadUpOnce()) strafeSpeed = Math.min(strafeSpeed + 0.01, 1.0);
            else if(controller1.dpadDown()) strafeSpeed = Math.max(strafeSpeed - 0.01, -1.0);

            if(controller1.dpadUpOnce()) rotationalSpeed = Math.min(rotationalSpeed + 0.01, 1.0);
            else if(controller1.dpadDown()) rotationalSpeed = Math.max(rotationalSpeed - 0.01, -1.0);

            opmode.motorUtility.setDriveForSimpleMecanum(strafeSpeed, -forwardSpeed, rotationalSpeed, 0);

            opmode.telemetry.addData("Speed: ", opmode.df.format(forwardSpeed) + ", " + opmode.df.format(strafeSpeed) + ", " + opmode.df.format(rotationalSpeed) + "\n");

            for (MotorTypes type : MotorTypes.values()) {
                opmode.telemetry.addLine(type.name());
                for (Motors motor : Motors.values()) {
                    if(type != motor.getType()) continue;
                    opmode.telemetry.addData(motor.name() + ": ", opmode.motorUtility.getEncoderValue(motor));
                }
            }
        }
    }

    class Servo_Tester extends Executive.StateBase {

        HashMap<Servos, Double> servoPositions = new HashMap<>();
        int servoIndex = 0;
        int maxServoIndex;
        int inputDivider = 10;
        double nextServoPosition;
        Servos currentServo;
        boolean disableClawServoTest = true;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opmode.telemetry.clear();

            for (Servos servo : Servos.values()) {
                try {
                    double previousServoPosition = opmode.servoUtility.getAngle(servo);
                    servoPositions.put(servo, previousServoPosition);
                } catch (Exception e) {
                    opmode.telemetry.addData("Servo Missing: ", servo.name());
                }
            }
            maxServoIndex = servoPositions.size()-1;
        }

        @Override
        public void update() {
            super.update();

            // Change state back to Menu
            if(controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            }

            // Select Index of servo to control.
            if(controller1.dpadUpOnce() && servoIndex < maxServoIndex) ++servoIndex;
            if(controller1.dpadDownOnce() &&  servoIndex > 0) --servoIndex;

            // Add a customizable controller input divider for more precise testing.
            if(controller1.dpadRightOnce()) inputDivider *= 10;
            if(controller1.dpadLeftOnce()) inputDivider /=  10;

            // Get the current servo that is selected and move it to the new position
            currentServo = Servos.values()[servoIndex];
            nextServoPosition = Range.clip(-controller1.left_stick_y / inputDivider + servoPositions.get(currentServo), -1, 1);
            servoPositions.put(currentServo, nextServoPosition);

            //Move Servo to position stored in servoPositions HashMap
            for (Servos servo : Servos.values()) {
                try {
                    opmode.servoUtility.setAngle(servo, servoPositions.get(servo));

                } catch (Exception e) {
                    opmode.telemetry.addData("Error couldn't set servo position for:  ", servo + ", " + opmode.df.format(servoPositions.get(servo)));
                }
            }
            opmode.telemetry.addData("Input Divider: ", inputDivider)
                    .addData("Servo: ", servoIndex + ", " + currentServo)
                    .addData("Servo Angle: ", opmode.df.format(servoPositions.get(currentServo)));
        }
    }

    class Shooter extends Executive.StateBase<AutoOpmode> {
        ElapsedTimer timer = new ElapsedTimer();
        int index = 1;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
        }

        @Override
        public void update() {
            super.update();
            if(index < 4) {
                opmode.motorUtility.setPower(Motors.LAUNCHER, launcherSpeed);
                if(timer.seconds() < launchDelay) {
                    opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                } else if(timer.seconds() < launchDelay + servoDelay) {
                    opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
                } else {
                    index++;
                    timer.reset();
                }
            } else {
                opmode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
                opmode.motorUtility.setPower(Motors.LAUNCHER, 0);
                nextState(DRIVE, new Start_Menu());
            }
        }
    }

    class Auto extends Executive.StateBase<AutoOpmode> {
        RobotStateContext robotStateContext;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opmode.telemetry.clear();
            robotStateContext = new RobotStateContext(opmode, teamColor, startPosition);
            robotStateContext.init();
        }

        @Override
        public void update() {
            super.update();
            robotStateContext.update();
            opmode.telemetry.addData("Sub State: ", robotStateContext.getCurrentState());
            if(controller1.startOnce()) stateMachine.changeState(DRIVE, new Start_Menu());
        }
    }

    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opmode.motorUtility.stopAllMotors();
        }
    }

    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
