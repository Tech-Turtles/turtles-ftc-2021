package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;

import static org.firstinspires.ftc.teamcode.Utility.Configuration.*;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double drivespeed = 1.0;
    public static double launchspeed = 0.6;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double rotationSpeed = 1.0;
    public static boolean powershotMode = false;
    public static double highGoalSpeed = 0.61;
    public static double powerShotSpeed = 0.55;

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
        mecanumNavigation = new MecanumNavigation(this, Configuration.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));

        imuUtil = new IMUUtilities(this, IMU.IMU1.getName());
        imuUtil.setCompensatedHeading(0);
    }

    @Override
    public void loop() {
        super.loop();

        motorUtility.setDriveForSimpleMecanum(
                primary.left_stick_x  * (drivespeed * precisionMode),
                primary.left_stick_y  * (drivespeed * precisionMode),
                primary.right_stick_x * (drivespeed * precisionMode) * rotationSpeed,
                primary.right_stick_y * (drivespeed * precisionMode));
        mecanumNavigation.update();
        imuUtil.update();

        if(primary.YOnce()) {
            imuUtil.setCompensatedHeading(0);
            mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
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

        if(secondary.dpadUpOnce()) {
            launchspeed = Math.min(launchspeed + 0.05, 1.0);
        } else if(secondary.dpadDownOnce()) {
            launchspeed = Math.max(launchspeed - 0.05, 0);
        }

        if(secondary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.LAUNCHER, launchspeed);
        }
//       // Button to reverse the launcher in case we have trouble with rings not being pushed enough to launch
//        else if(secondary.left_trigger > deadzone) {
//            motorUtility.setPower(Motors.LAUNCHER, -launchspeed);
//        }
        else {
            motorUtility.setPower(Motors.LAUNCHER, 0f);
        }

        if(secondary.rightBumper()) {
            servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
        } else {
            servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
        }

        if(secondary.AOnce()) {
            launchspeed = powershotMode ? powerShotSpeed : highGoalSpeed;
            powershotMode = !powershotMode;
        }


        telemetry.addLine("----Navigation----");
        mecanumNavigation.displayPosition();
        telemetry.addData("IMU heading:         ", imuUtil.getCompensatedHeading());
        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("Powershot mode:      ", powershotMode);
        telemetry.addLine("----Launcher----");
        telemetry.addData("Launcher speed:      ", df.format(launchspeed));
        telemetry.addData("Launch velocity:     ", motorUtility.getVelocity(Motors.LAUNCHER));
        telemetry.addData("Hopper position:     ", servoUtility.getAngle(Servos.HOPPER));
        telemetry.addLine();
        telemetry.addData("Drive speed:         ", df.format(drivespeed));
        telemetry.addData("Precision speed:     ", df.format(precisionPercentage));
        telemetry.addData("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
    }
}