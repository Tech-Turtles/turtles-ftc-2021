package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.AutoDrive;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.Mecanum;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;

import java.text.DecimalFormat;
import java.util.HashMap;

import org.firstinspires.ftc.teamcode.HardwareTypes.*;
import org.firstinspires.ftc.teamcode.Menu.InteractiveInitialization;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utility.Vision.UGCoffeeDetector;
import org.firstinspires.ftc.teamcode.Utility.Vision.UGContourRingDetector;
import org.firstinspires.ftc.teamcode.Utility.Vision.UGContourRingPipeline;

/**
 * @author Christian
 * Revamped RobotHardware, better readability, new methods, more organized, easier to build upon.
 */
public class RobotHardware extends OpMode {

    private static final HashMap<Motors, DcMotorEx> motors = new HashMap<>();
    private static final HashMap<Servos, Servo> servos = new HashMap<>();

    public final MotorUtility motorUtility = new MotorUtility();
    public final ServoUtility servoUtility = new ServoUtility();

    public DecimalFormat df = new DecimalFormat("0.00");
    public DecimalFormat df_precise = new DecimalFormat("0.0000");

    public IMUUtilities imuUtil;

    public InteractiveInitialization initMenu;

    protected LynxModule expansionHub1;
    protected LynxModule expansionHub2;

    public final ElapsedTimer period = new ElapsedTimer();

    public Controller primary, secondary;

    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;

    public UGCoffeeDetector ringDetector;

    public class MotorUtility {

        private DcMotorEx m;

        /**
         * Do not make this function public.
         * If the motor is null, any function run on it will cause a fatal error.
         */
        private DcMotorEx getMotor(Motors motor) {
            m = motors.get(motor);
            if (m == null)
                telemetry.addData("Motor Missing", motor.name());
            return m;
        }

        public double getPower(Motors motor) {
            m = getMotor(motor);
            if (m != null)
                return m.getPower();
            return 0;
        }

        public void setPower(Motors motor, double power) {
            m = getMotor(motor);
            if (m != null)
                m.setPower(power);
        }

        public int getEncoderValue(Motors motor) {
            m = getMotor(motor);
            if (m == null) return -1;
            return m.getCurrentPosition();
        }

        public double getVelocity(Motors motor) {
            m = getMotor(motor);
            if (m == null) return 0;
            return m.getVelocity();
        }

        public void stopAllMotors() {
            for (Motors motor : Motors.values())
                setPower(motor, 0f);
        }

        public void stopResetAllMotors() {
            motors.forEach((k, v) -> v.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        }

        public void stopResetMotor(Motors motor) {
            m = getMotor(motor);
            if (m != null)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void setTypeMotorsRunmode(MotorTypes type, DcMotor.RunMode runMode) {
            for (Motors motor : Motors.values()) {
                if (!motor.getType().equals(type)) continue;
                m = getMotor(motor);
                if (m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setMotorsRunMode(DcMotor.RunMode runMode, Motors... motors) {
            for (Motors motor : motors) {
                m = getMotor(motor);
                if (m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setTypeMotorsZeroPowerBehavior(MotorTypes type, DcMotor.ZeroPowerBehavior behavior) {
            for (Motors motor : Motors.values()) {
                if (!motor.getType().equals(type)) continue;
                m = getMotor(motor);
                if (m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior, Motors... motors) {
            for (Motors motor : motors) {
                m = getMotor(motor);
                if (m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        private void initializeMotors() {
            stopResetAllMotors();
            motors.forEach((k, v) -> {
                v.setMode(k.getRunMode());
                v.setZeroPowerBehavior(k.getZeroPowerBehavior());
                v.setDirection(k.getDirection());
            });
        }

        /**
         * Sets the drive chain power.
         *
         * @param left  The power for the left two motors.
         * @param right The power for the right two motors.
         */
        public void setDriveForTank(double left, double right) {
            setPower(Motors.FRONT_LEFT, left);
            setPower(Motors.BACK_LEFT, left);
            setPower(Motors.FRONT_RIGHT, right);
            setPower(Motors.BACK_RIGHT, right);
        }

        /**
         * Apply motor power matching the wheels object.
         *
         * @param wheels Provides all four mecanum wheel powers, [-1, 1].
         */
        public void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
            setPower(Motors.FRONT_LEFT, wheels.frontLeft);
            setPower(Motors.BACK_LEFT, wheels.backLeft);
            setPower(Motors.FRONT_RIGHT, wheels.frontRight);
            setPower(Motors.BACK_RIGHT, wheels.backRight);
        }

        public void setDriveForMecanumCommand(Mecanum.Command command) {
            Mecanum.Wheels wheels = Mecanum.commandToWheels(command);
            setDriveForMecanumWheels(wheels);
        }

        /**
         * Sets mecanum drive chain power using simplistic calculations.
         *
         * @param leftStickX  Unmodified Gamepad leftStickX inputs.
         * @param leftStickY  Unmodified Gamepad leftStickY inputs.
         * @param rightStickX Unmodified Gamepad rightStickX inputs.
         * @param rightStickY Unmodified Gamepad rightStickY inputs.
         */
        public void setDriveForSimpleMecanum(double leftStickX, double leftStickY,
                                             double rightStickX, double rightStickY) {
            Mecanum.Wheels wheels = Mecanum.simpleJoystickToWheels(leftStickX, leftStickY, rightStickX, rightStickY);
            setDriveForMecanumWheels(wheels);
        }
    }

    public class ServoUtility {
        Servo s;

        private Servo getServo(Servos servo) {
            s = servos.get(servo);
            if (s == null) {
                telemetry.addData("Servo Missing", servo.name());
            }
            return s;
        }

        public void setAngle(Servos servo, double pos) {
            s = getServo(servo);
            if (s != null)
                s.setPosition(pos);
        }

        public double getAngle(Servos servo) {
            s = getServo(servo);
            if (s != null)
                return s.getPosition();
            return -1;
        }

        public boolean moveServoAtRate(Servos servo, double targetPos, double rate) {
            boolean isMovementDone = false;
            double distanceThreshold = 0.05;
            rate = Range.clip(rate, 0, 10);
            targetPos = Range.clip(targetPos, 0, 1);
            double currentPosition = getAngle(servo);
            double distance = targetPos - currentPosition;
            double direction = targetPos > currentPosition ? 1 : -1;
            double nextPosition;
            if (Math.abs(distance) > distanceThreshold) {
                nextPosition = rate * direction * period.getLastPeriodSec() + currentPosition;
            } else {
                nextPosition = targetPos;
                isMovementDone = true;
            }
            nextPosition = Range.clip(nextPosition, 0, 1);
            setAngle(servo, nextPosition);

            return isMovementDone;
        }
    }

    public enum StartPosition {
        WALL,
        CENTER
    }

    public void loadVision(boolean debug) {
        ringDetector = new UGCoffeeDetector(hardwareMap, Webcam.WEBCAM_1.getName(), telemetry, debug);
        ringDetector.init();
    }

    public void clearHubCache() {
        try {
            expansionHub1.clearBulkCache();
        } catch (Exception e) {
            telemetry.addLine("Error: " + e.getMessage());
        }
        try {
            expansionHub2.clearBulkCache();
        } catch (Exception e) {
            telemetry.addLine("Error: " + e.getMessage());
        }
    }

    /**
     * Updates the mecanumNavigation heading from the imu heading.
     * This function forces the IMU to refresh immediately.
     */
    public void updateMecanumHeadingFromGyroNow() {
        if(imuUtil == null || mecanumNavigation == null) return;
        imuUtil.updateNow();
        MecanumNavigation.Navigation2D currentPosition = mecanumNavigation.getCurrentPosition();
        currentPosition.theta = Math.toRadians(imuUtil.getCompensatedHeading());
        mecanumNavigation.setCurrentPosition(currentPosition);
    }

    public double getTime() {
        return time;
    }

    @Override
    public void init() {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            expansionHub1 = hardwareMap.get(LynxModule.class, ExpansionHubs.HUB1.getHub());
            expansionHub2 = hardwareMap.get(LynxModule.class, ExpansionHubs.HUB2.getHub());

            expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (IllegalArgumentException | NullPointerException e) {
            telemetry.addLine(e.getMessage());
        }

        for (Motors m : Motors.values()) {
            try {
                motors.put(m, hardwareMap.get(DcMotorEx.class, m.getConfigName()));
            } catch (IllegalArgumentException ignore) {
                telemetry.addData("Motor Missing", m.name());
            }
        }
        new MotorUtility().initializeMotors();

        for (Servos s : Servos.values()) {
            try {
                Servo servo = hardwareMap.get(Servo.class, s.getConfigName());
                servos.put(s, servo);
                servo.setDirection(s.getDirection());
            } catch (IllegalArgumentException ignore) {
                telemetry.addData("Servo Missing", s.name());
            }
        }

        initMenu = new InteractiveInitialization(this);
        primary = new Controller(gamepad1);
        secondary = new Controller(gamepad2);
        motorUtility.stopAllMotors();
        period.reset();
    }

    @Override
    public void init_loop() {
        initMenu.loop();
        period.updatePeriodTime();
        primary.update();
        secondary.update();
    }

    @Override
    public void start() {
        motorUtility.stopAllMotors();
        period.reset();
    }

    @Override
    public void loop() {
        period.updatePeriodTime();
        primary.update();
        secondary.update();
    }

    /**
     * Stops all motors and calls requestOpModeStop() to end the opmode
     */
    @Override
    public void stop() {
        motorUtility.stopAllMotors();
    }
}