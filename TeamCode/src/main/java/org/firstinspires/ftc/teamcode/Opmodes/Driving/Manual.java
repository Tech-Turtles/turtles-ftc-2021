package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Waypoints;

import static org.firstinspires.ftc.teamcode.Opmodes.Driving.Manual.DashboardVariables.*;

@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    @Config
    static class DashboardVariables {
        public static double drivespeed = 1.0;
        public static double launchspeed = 1.0;
    }

    private final double deadzone = 0.49f;

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
        Waypoints waypoints = new Waypoints(this);
        waypoints.test();
    }
    @Override public void loop() {
        super.loop();

        motorUtility.setDriveForSimpleMecanum(primary.left_stick_x * drivespeed, primary.left_stick_y * drivespeed,
                primary.right_stick_x * drivespeed, primary.right_stick_y * drivespeed);

        if(primary.leftBumperOnce()) {
            drivespeed = drivespeed <= 0.0 ? 0.0 : drivespeed - 0.1;
        } else if(primary.rightBumperOnce()) {
            drivespeed = drivespeed >= 1.0 ? 1.0 : drivespeed + 0.1;
        }

        if(primary.dpadUpOnce()) {
            launchspeed = launchspeed >= 1.0 ? 1.0 : launchspeed + 0.01;
        } else if(primary.dpadDownOnce()) {
            launchspeed = launchspeed <= 0.0 ? 0.0 : launchspeed - 0.01;
        }

        if(primary.A()) {
            motorUtility.setPower(Motors.LAUNCHER, launchspeed);
        } else {
            motorUtility.setPower(Motors.LAUNCHER, 0f);
        }

        if(primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, 1f);
        } else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -1f);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0f);
        }

        telemetry.addData("Launch velocity A:   ", getLauncherTicksPerSecond());
        telemetry.addData("Launch velocity B:   ", motorUtility.getVelocity(Motors.LAUNCHER));
        telemetry.addData("Drive speed:         ", df.format(drivespeed));
        telemetry.addData("Launcher speed:      ", df.format(launchspeed));
        telemetry.addData("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
    }

    int launcherTicks, launcherPreviousTicks, launcherDeltaTicks = 0;
    double ticksPerSecond = 0;

    int getLauncherTicksPerSecond() {
        launcherTicks = motorUtility.getEncoderValue(Motors.LAUNCHER);
        launcherDeltaTicks = launcherTicks - launcherPreviousTicks;
        ticksPerSecond = launcherDeltaTicks / period.getLastPeriodSec();
        launcherPreviousTicks = launcherTicks;
        return (int) ticksPerSecond;
    }
}