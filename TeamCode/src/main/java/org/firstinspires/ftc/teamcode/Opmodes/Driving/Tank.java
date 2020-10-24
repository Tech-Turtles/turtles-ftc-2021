package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@TeleOp(name="Tank Drive", group="E")
@Disabled
public class Tank extends RobotHardware {

    private final StringBuilder builder = new StringBuilder();

    @Override
    public void loop() {
        super.loop();
        builder.setLength(0);

        double leftPower  = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0) ;
        double rightPower = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x, -1.0, 1.0) ;
        motorUtility.setDriveForTank(leftPower, rightPower);

        for (Motors motor : Motors.values()) {
            if(motor.getType() != MotorTypes.DRIVE) continue;
            builder.append(motor.name())
                    .append(": ")
                    .append(motorUtility.getEncoderValue(motor))
                    .append("\n");
        }

        telemetry.addLine(builder.toString());
    }
}
