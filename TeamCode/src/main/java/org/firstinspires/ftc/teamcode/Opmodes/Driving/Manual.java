package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.*;

@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public final StringBuilder driveMenu = new StringBuilder();
    private double drivespeed = 1.0;
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
        driveMenu.append(TelemetryTools.setHeader(1, "Drive Menu"));

        primary.update();
        secondary.update();

        motorUtility.setDriveForSimpleMecanum(primary.left_stick_x * drivespeed, primary.left_stick_y * drivespeed,
                primary.right_stick_x * drivespeed, primary.right_stick_y * drivespeed);

        if(primary.leftBumperOnce()) {
            drivespeed = drivespeed <= 0.0 ? 0.0 : drivespeed - 0.1;
        } else if(primary.rightBumperOnce()) {
            drivespeed = drivespeed >= 1.0 ? 1.0 : drivespeed + 0.1;
        }

        if(primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, 1f);
        } else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -1f);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0f);
        }
        
        driveMenu.append("Drive speed: ").append(df.format(drivespeed));
        driveMenu.append("\n");
        driveMenu.append("Loop time: ").append(period.getAveragePeriodSec());
        driveMenu.append("\n\n");
        telemetry.addLine(driveMenu.toString());
    }
}
