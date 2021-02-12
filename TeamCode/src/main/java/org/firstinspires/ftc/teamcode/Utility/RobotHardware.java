package org.firstinspires.ftc.teamcode.Utility;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.AutoDrive;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.Mecanum;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;

import java.text.DecimalFormat;
import java.util.HashMap;

import org.firstinspires.ftc.teamcode.HardwareTypes.*;
import org.firstinspires.ftc.teamcode.Menu.InteractiveInitialization;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.UGCoffeeDetector;

/**
 * @author Christian
 * Revamped RobotHardware, better readability, new methods, more organized, easier to build upon.
 */
public class RobotHardware extends OpMode {

    private static final HashMap<Motors, DcMotorEx> motors = new HashMap<>();
    private static final HashMap<Servos, Servo> servos = new HashMap<>();
    private static final HashMap<ContinuousServo, CRServo> crServos = new HashMap<>();
    private static final HashMap<ColorSensor, RevColorSensorV3> colorSensors = new HashMap<>();

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

    public SampleMecanumDrive mecanumDrive;

    public static Pose2d lastPosition = new Pose2d(0,0,0);
    public static Pose2d shoot1;
    public static Pose2d shoot2;
    public static Pose2d shoot3;
    public static int lastWobblePosition = 0;


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
            getMotor(motor);
            if (m == null)
                return 0;
            return m.getPower();
        }

        public void setPower(Motors motor, double power) {
            getMotor(motor);
            if (m != null)
                m.setPower(power);
        }

        public int getEncoderValue(Motors motor) {
            getMotor(motor);
            if (m == null) return -1;
            return m.getCurrentPosition();
        }

        public double getVelocity(Motors motor) {
            getMotor(motor);
            if (m == null) return 0;
            return m.getVelocity();
        }

        public void stopAllMotors() {
            for (Motors motor : Motors.values())
                setPower(motor, 0);
        }

        public void stopResetAllMotors() {
            motors.forEach((k, v) -> v.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        }

        public void stopResetMotor(Motors motor) {
            getMotor(motor);
            if (m != null)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void setPIDFCoefficients(DcMotor.RunMode runmode, PIDFCoefficients pidfCoefficients, Motors... motors) {

        }

        public void setTypeMotorsRunmode(MotorTypes type, DcMotor.RunMode runMode) {
            for (Motors motor : Motors.values()) {
                if (!motor.getType().equals(type)) continue;
                getMotor(motor);
                if (m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setMotorsRunMode(DcMotor.RunMode runMode, Motors... motors) {
            for (Motors motor : motors) {
                getMotor(motor);
                if (m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setTypeMotorsZeroPowerBehavior(MotorTypes type, DcMotor.ZeroPowerBehavior behavior) {
            for (Motors motor : Motors.values()) {
                if (!motor.getType().equals(type)) continue;
                getMotor(motor);
                if (m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior, Motors... motors) {
            for (Motors motor : motors) {
                getMotor(motor);
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


        /**
         * @param motor The motor that will be driven
         * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
         * @param power The power at which the robot will be driven
         * @param rampThreshold The position when the robot will start slowing the motor down before its destination
         * @return Returns whether or not the motor arrived to the specified position
         */
        public boolean goToPosition(Motors motor, int targetTicks, double power, double rampThreshold) {
            power = Range.clip(Math.abs(power), 0, 1);
            int poweredDistance = 0;
            int arrivedDistance = 50;
            double maxRampPower = 1.0;
            double minRampPower = 0.0;
            int errorSignal = getEncoderValue(motor) - targetTicks;
            double direction = errorSignal > 0 ? -1.0 : 1.0;
            double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, maxRampPower, minRampPower);

            if (Math.abs(errorSignal) >= poweredDistance) {
                setPower(motor, direction * power * rampDownRatio);
            } else {
                setPower(motor, 0);
            }

            return Math.abs(errorSignal) <= arrivedDistance;
        }

        /**
         * @param motor The motor that will be driven
         * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
         * @param power The power at which the robot will be driven
         * @return Returns whether or not the motor arrived to the specified position
         */
        public boolean goToPosition(Motors motor, int targetTicks, double power) {
            int rampDistanceTicks = 400;
            return goToPosition(motor, targetTicks, power, rampDistanceTicks);
        }
    }

    public class ServoUtility {
        Servo s;
        CRServo cr;

        private Servo getServo(Servos servo) {
            s = servos.get(servo);
            if (s == null) {
                telemetry.addData("Servo Missing", servo.name());
            }
            return s;
        }

        private CRServo getContinuousServo(ContinuousServo servo) {
            cr = crServos.get(servo);
            if (cr == null) {
                telemetry.addData("CRServo Missing", servo.name());
            }
            return cr;
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

        public void setPower(ContinuousServo continuousServo, double power) {
            cr = getContinuousServo(continuousServo);
            if(cr == null) return;
            cr.setPower(power);
        }
    }

    public RevColorSensorV3 getColorSensor(ColorSensor colorSensor) {
        RevColorSensorV3 revColorSensorV3 = colorSensors.get(colorSensor);
        if(revColorSensorV3 == null)
            telemetry.addData("Sensor Missing", colorSensor.name());
        return revColorSensorV3;
    }

    public double getDistance(RevColorSensorV3 colorSensor) {
        if(colorSensor == null) return -1;
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);
    }

    public void loadVision(boolean debug) {
        ringDetector = new UGCoffeeDetector(hardwareMap, Webcam.WEBCAM_1.getName(), telemetry, debug);
        ringDetector.init();
    }

    public void clearHubCache() {
        try {
            expansionHub1.clearBulkCache();
            expansionHub2.clearBulkCache();
        } catch (Exception e) {
            telemetry.addLine("Error: " + e.getMessage());
        }
    }

    /**
     * Updates the mecanumNavigation heading from the imu heading.
     * This function forces the IMU to refresh immediately.
     */
    public void updateMecanumHeadingFromGyroNow(IMUUtilities imuUtil, MecanumNavigation mecanumNavigation) {
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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            expansionHub1 = hardwareMap.get(LynxModule.class, ExpansionHubs.HUB1.getHub());
            expansionHub2 = hardwareMap.get(LynxModule.class, ExpansionHubs.HUB2.getHub());

            expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (IllegalArgumentException | NullPointerException e) {
            telemetry.addLine("Error: " + e.getMessage());
        }
        clearHubCache();

        for (Motors m : Motors.values()) {
            try {
                motors.put(m, hardwareMap.get(DcMotorEx.class, m.getConfigName()));
            } catch (IllegalArgumentException ignore) {}
        }
        new MotorUtility().initializeMotors();

        for (Servos s : Servos.values()) {
            try {
                Servo servo = hardwareMap.get(Servo.class, s.getConfigName());
                servos.put(s, servo);
                servo.setDirection(s.getDirection());
            } catch (IllegalArgumentException ignore) {}
        }

        for (ContinuousServo c : ContinuousServo.values()) {
            try {
                CRServo crServo = hardwareMap.get(CRServo.class, c.getConfigName());
                crServos.put(c, crServo);
                crServo.setDirection(c.getDirection());
            } catch (IllegalArgumentException ignore) {}
        }

        for (ColorSensor c : ColorSensor.values()) {
            try {
                RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, c.getConfig());
                colorSensors.put(c, colorSensor);
                colorSensor.enableLed(false);
            } catch (IllegalArgumentException ignore) {}
        }

//        initMenu = new InteractiveInitialization(this);
        primary = new Controller(gamepad1);
        secondary = new Controller(gamepad2);
        motorUtility.stopAllMotors();
        period.reset();
    }

    @Override
    public void init_loop() {
        clearHubCache();
//        initMenu.loop();
        period.updatePeriodTime();
        primary.update();
        secondary.update();
    }

    @Override
    public void start() {
        clearHubCache();
        motorUtility.stopAllMotors();
        period.reset();
    }

    @Override
    public void loop() {
        clearHubCache();
        period.updatePeriodTime();
        primary.update();
        secondary.update();
    }
    
    /**
     * Stops all motors and calls requestOpModeStop() to end the opmode
     */
    @Override
    public void stop() {
        clearHubCache();
        try {
            lastPosition = mecanumDrive.getPoseEstimate();
            lastWobblePosition = motorUtility.getEncoderValue(Motors.WOBBLE_ARM);
        } catch (Exception e) {
            Log.wtf("Unable to save positions", e.getMessage());
        }
        motorUtility.stopAllMotors();
        for (ContinuousServo servo : ContinuousServo.values())
            servoUtility.setPower(servo, 0);
    }
}