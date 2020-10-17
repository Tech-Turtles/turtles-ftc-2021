package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Controller;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.TelemetryTools;

@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    private final StringBuilder driveMenu = new StringBuilder();
    private double drivespeed = 1.0;

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

        if(primary.right_trigger > 0.6f) {
            motorUtility.setPower(Motors.LAUNCHER, 1f);
        } else {
            motorUtility.setPower(Motors.LAUNCHER, 0f);
        }
        
        driveMenu.append(TelemetryTools.setHeader(6, "Drive speed: " + TelemetryTools.setFontColor("Grey", Double.toString(drivespeed))));

        for (MotorTypes type : MotorTypes.values()) {
            driveMenu.append(TelemetryTools.setHeader(4, type.name())).append("\n");
            for (Motors motor : Motors.values()) {
                if(type != motor.getType()) continue;
                driveMenu.append(motor.name())
                        .append(": ")
                        .append(motorUtility.getEncoderValue(motor))
                        .append("\n");
            }
        }

        driveMenu.append("Loop time: ").append(timer.getAveragePeriodSec());

        telemetry.addLine(driveMenu.toString());
    }

}
