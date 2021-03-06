package org.firstinspires.ftc.teamcode.Utility.Roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Math.LauncherControl;
import org.firstinspires.ftc.teamcode.Utility.Math.ListMath;

import java.util.ArrayList;

@Config
@TeleOp(name="SingleMotorPIDFTest")
public class SingleMotorPIDF extends LinearOpMode {
    public static double delay = 5.0; // sec
    public static double launchSpeed = 0.56;
    boolean moving = true;
    private VoltageSensor batteryVoltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private ArrayList<Double> velocityHistory = new ArrayList<>();
    DcMotorEx launcher;
    public static int isErrorLow = 0;
    public static double average = 0;
    public static double deviation = 0;
    public static double Kp = 10;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        for (int i = 0; i < 99; i++) {
            velocityHistory.add(i, 0.0);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            ListMath.addRemoveN(velocityHistory, launcher.getVelocity(), 100);
            average = Math.abs(ListMath.average(velocityHistory, Configuration.launchErrorWindowSteps) - Configuration.LAUNCHER_THEORETICAL_MAX * launchSpeed);
            deviation = ListMath.standardDeviation(velocityHistory, Configuration.launchErrorWindowSteps);
            if (timer.seconds() > delay) {
                launcher.setPower(moving ? launchSpeed : 0);
                moving = !moving;
                setPIDFCoefficients(launcher, DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
                timer.reset();
            }

            isErrorLow = LauncherControl.isErrorLow(velocityHistory, Configuration.LAUNCHER_THEORETICAL_MAX * launchSpeed) ? 2000 : 0;
            telemetry.addData("isErrorLow", isErrorLow);
            telemetry.addData("average", average);
            telemetry.addData("deviation", deviation);
            telemetry.addData("targetVelocity", Configuration.LAUNCHER_THEORETICAL_MAX * launchSpeed);
            telemetry.addData("measuredVelocity", launcher.getVelocity());
            telemetry.addData(
                    "error", Configuration.LAUNCHER_THEORETICAL_MAX * launchSpeed - launcher.getVelocity()
            );
            telemetry.addData("voltage", batteryVoltageSensor.getVoltage());

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