package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utility.Configuration.*;

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
    public static int wobbleUp = 1400;
    public static int wobbleStore = 0;
    public static int wobbleDown = 3550;
    public static double wobblePower = 1.0;
    private WobbleStates wobbleState = WobbleStates.MANUAL;
    private boolean wobbleArrived = false;

    private enum WobbleStates {
        MANUAL,
        UP,
        DOWN,
        STORE
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        imuUtil = new IMUUtilities(this, IMU.IMU1.getName());
        imuUtil.setCompensatedHeading(0);
    }

    @Override
    public void loop() {
        super.loop();

        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * linearSpeed * precisionMode,
                        -gamepad1.left_stick_x * lateralSpeed * precisionMode,
                        -gamepad1.right_stick_x * rotationSpeed * precisionMode
                )
        );

        mecanumDrive.update();
        imuUtil.update();

        if(primary.YOnce()) {
            imuUtil.setCompensatedHeading(0);
            mecanumDrive.clearEstimatedPose();
        }

        if(primary.AOnce()) {
            precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;
        }

        if(primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, 1f);
        } else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -1f);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0f);
        }

        if(secondary.XOnce())
            wobbleState = WobbleStates.STORE;
        else if(secondary.BOnce())
            wobbleState = WobbleStates.DOWN;
        else if(secondary.YOnce())
            wobbleState = WobbleStates.UP;

        switch (wobbleState) {
            case UP:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, wobbleUp, wobblePower);
                break;
            case DOWN:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, wobbleDown, wobblePower);
                break;
            case STORE:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, wobbleStore, wobblePower);
                break;
            case MANUAL:
                motorUtility.setPower(Motors.WOBBLE_ARM, -secondary.right_stick_y * wobblePower);
        }
        wobbleState = wobbleArrived ? WobbleStates.MANUAL : wobbleState;

        servoUtility.setPower(ContinuousServo.WOBBLE_LEFT, secondary.left_stick_y);
        servoUtility.setPower(ContinuousServo.WOBBLE_RIGHT, secondary.left_stick_y);

        if(secondary.dpadUpOnce()) {
            launchspeed = Math.min(launchspeed + 0.01, 1.0);
        } else if(secondary.dpadDownOnce()) {
            launchspeed = Math.max(launchspeed - 0.01, 0);
        }

        if(secondary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.LAUNCHER, launchspeed);
        } else {
            motorUtility.setPower(Motors.LAUNCHER, 0f);
        }

        if(secondary.rightBumper()) {
            servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
        } else {
            servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
        }

        if(secondary.AOnce()) {
            powershotMode = !powershotMode;
            launchspeed = powershotMode ? powerShotSpeed : highGoalSpeed;
        }

        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("Launcher speed:      ", df.format(launchspeed));
        telemetry.addData("Powershot mode:      ", powershotMode);
        try {
            telemetry.addData("Wobble Distance: ", getDistance(getColorSensor(ColorSensor.WOBBLE_SENSOR)));
        } catch(NullPointerException ignore) {}
        telemetry.addLine();
        telemetry.addLine("----Launcher----");
        telemetry.addData("Launch velocity:     ", motorUtility.getVelocity(Motors.LAUNCHER));
        telemetry.addData("Hopper position:     ", servoUtility.getAngle(Servos.HOPPER));
        telemetry.addLine();
        telemetry.addLine("----Navigation----");
        telemetry.addData("X:                   ", poseEstimate.getX());
        telemetry.addData("Y:                   ", poseEstimate.getY());
        telemetry.addData("Heading:             ", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("IMU heading:         ", imuUtil.getCompensatedHeading());
        telemetry.addData("Drive speed:         ", df.format(drivespeed));
        telemetry.addData("Precision speed:     ", df.format(precisionPercentage));
        telemetry.addData("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        telemetry.update();
    }
}