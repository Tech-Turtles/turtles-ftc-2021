package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utility.TelemetryTools;

/**
 * @author Christian
 * An opmode that shows data about commonly used things, including motors, servos, IMU.
 */
@TeleOp(name="Debug", group="B")
public class Debug extends Manual {

    IMUUtilities imuUtil;

    @Override
    public void init() {
        super.init();
        imuUtil = new IMUUtilities(this, "IMU_1", IMUUtilities.ImuMode.FAST_HEADING_ONLY);
        telemetry.addLine("\n\nDebug Initialized");
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

        for (MotorTypes type : MotorTypes.values()) {
            driveMenu.append(TelemetryTools.setHeader(4, type.name())).append("\n");
            for (Motors motor : Motors.values()) {
                if(type != motor.getType()) continue;
                driveMenu.append(motor.name()).append(": ")
                        .append(motorUtility.getEncoderValue(motor))
                        .append("\n");
            }
        }

        for (Servos servo : Servos.values()) {
            driveMenu.append(servo.name()).append(": ")
                    .append(servoUtility.getAngle(servo))
                    .append("\n");
        }

        driveMenu.append("Period Average (sec)").append(df_precise.format(period.getAveragePeriodSec()))
                .append("Period Max (sec)").append(df_precise.format(period.getMaxPeriodSec()));

        if(imuUtil.imu != null) {
            imuUtil.updateNow();
            imuUtil.displayTelemetry();
        }

        telemetry.addLine(driveMenu.toString());
    }
}
