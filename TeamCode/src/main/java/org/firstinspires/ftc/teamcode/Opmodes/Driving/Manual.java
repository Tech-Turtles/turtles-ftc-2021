package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utility.Controller;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

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

        primary.update();
        secondary.update();

        motorUtility.setDriveForSimpleMecanum(primary.left_stick_x, primary.left_stick_y, primary.right_stick_x, primary.right_stick_y);
    }

}
