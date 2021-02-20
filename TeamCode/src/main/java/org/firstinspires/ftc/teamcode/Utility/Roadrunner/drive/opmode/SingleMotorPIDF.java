package org.firstinspires.ftc.teamcode.Utility.Roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.Configuration;

@Config
@TeleOp(name="SingleMotorPIDFTest")
@Disabled
public class SingleMotorPIDF extends LinearOpMode {
    public static double delay = 5.0; // sec
    public static double launchSpeed = 0.56;
    boolean moving = true;
    public static PIDFCoefficients pidfCoefficients;
    private VoltageSensor batteryVoltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    DcMotorEx launcher;
    public static double lastKp = 0;
    public static double lastKi = 0;
    public static double lastKd = 0;
    public static double lastKf = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidfCoefficients = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        lastKp = pidfCoefficients.p;
        lastKi = pidfCoefficients.i;
        lastKd = pidfCoefficients.d;
        lastKf = pidfCoefficients.f;

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            if (timer.seconds() > delay) {
                launcher.setPower(moving ? launchSpeed : 0);
                moving = !moving;
                timer.reset();
            }

            telemetry.addData("targetVelocity", Configuration.LAUNCHER_THEORETICAL_MAX * launchSpeed);
            telemetry.addData("measuredVelocity", launcher.getVelocity());
            telemetry.addData(
                    "error", Configuration.LAUNCHER_THEORETICAL_MAX * launchSpeed - launcher.getVelocity()
            );
            telemetry.addData("voltage", batteryVoltageSensor.getVoltage());

            if (lastKp != pidfCoefficients.p || lastKd != pidfCoefficients.d
                    || lastKi != pidfCoefficients.i || lastKf != pidfCoefficients.f) {
                setPIDFCoefficients(launcher, DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

                lastKp = pidfCoefficients.p;
                lastKi = pidfCoefficients.i;
                lastKd = pidfCoefficients.d;
                lastKf = pidfCoefficients.f;
            }

            telemetry.update();
        }
    }

    public void setPIDFCoefficients(DcMotorEx motor, DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        motor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
}