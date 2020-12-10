package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;

import static org.firstinspires.ftc.teamcode.Opmodes.Driving.Manual.DashboardVariables.*;

@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    @Config
    static class DashboardVariables {
        public static double drivespeed = 1.0;
        public static double launchspeed = 0.6;
        public static double hopperPosition = .0;
    }

    private final double deadzone = 0.49f;

    @Override
    public void init() {
        super.init();
        hopperPosition = servoUtility.getAngle(Servos.HOPPER);
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
                primary.left_stick_x  * drivespeed,
                primary.left_stick_y  * drivespeed,
                primary.right_stick_x * drivespeed,
                primary.right_stick_y * drivespeed);

        imuUtil.update();

        if(primary.A()) {
            motorUtility.setPower(Motors.LAUNCHER, launchspeed);
        } else {
            motorUtility.setPower(Motors.LAUNCHER, 0f);
        }

        if(primary.YOnce()) {
            imuUtil.setCompensatedHeading(0);
        }

        if(primary.rightBumper()) {
            servoUtility.setAngle(Servos.HOPPER, 0.23);
        } else {
            servoUtility.setAngle(Servos.HOPPER, 0.32);
        }

        if(primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, 1f);
        } else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -1f);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0f);
        }

        telemetry.addData("Launch velocity:     ", motorUtility.getVelocity(Motors.LAUNCHER));
        telemetry.addData("Drive speed:         ", df.format(drivespeed));
        telemetry.addData("Launcher speed:      ", df.format(launchspeed));
        telemetry.addData("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        telemetry.addData("IMU Heading:         ", imuUtil.getCompensatedHeading());
        telemetry.addData("Hopper Position:     ", servoUtility.getAngle(Servos.HOPPER));
    }
}