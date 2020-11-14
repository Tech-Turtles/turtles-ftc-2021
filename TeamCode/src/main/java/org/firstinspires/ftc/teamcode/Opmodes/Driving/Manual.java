package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.*;

@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public final StringBuilder driveMenu = new StringBuilder();
    private double drivespeed = 1.0;
    private double launchspeed = 1.0;
    private final double deadzone = 0.49f;

    @Override
    public void init() {
        super.init();
        primary = new Controller(gamepad1);
        secondary = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        driveMenu.setLength(0);
        driveMenu.append("Drive Menu");

        primary.update();
        secondary.update();

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

        driveMenu.append("Launch velocity A: ").append(getLauncherTicksPerSecond())
            .append("\n")
            .append("Launch velocity B:").append(motorUtility.getVelocity(Motors.LAUNCHER))
            .append("\n")
            .append("Drive speed: ").append(df.format(drivespeed))
            .append("\n")
            .append("Launcher speed: ").append(df.format(launchspeed))
            .append("\n")
            .append("Loop time: ").append(period.getAveragePeriodSec())
            .append("\n\n");
        telemetry.addLine(driveMenu.toString());
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