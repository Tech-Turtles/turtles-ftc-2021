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

        motorUtility.setDriveForSimpleMecanum(primary.left_stick_x, primary.left_stick_y, primary.right_stick_x, primary.right_stick_y);
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


        telemetry.addLine(driveMenu.toString());
    }

}
