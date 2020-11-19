package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;


public class AutoOpmode extends RobotHardware {

    AllianceColor robotColor;
    StartPosition robotStartPos;
    private Executive.RobotStateMachineContextInterface robotStateContext;


    @Autonomous(name="Wall Red", group="A-R")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.WALL;
            super.init();
        }
    }

    @Autonomous(name="Center Red", group="A-R")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.CENTER;
            super.init();
        }
    }

    @Autonomous(name="Wall Blue", group="A-B")
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.WALL;
            super.init();
        }
    }

    @Autonomous(name="Center Blue", group="A-B")
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.CENTER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        primary = new Controller(gamepad1);
        imuUtil = new IMUUtilities(this, IMU.IMU1.getName());
        robotStateContext = new RobotStateContext(AutoOpmode.this, robotColor, robotStartPos);
        telemetry.addData("Initialization: ", "Successful!");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        primary.update();
    }

    @Override
    public void start() {
        super.start();
        // Navigation and control
        mecanumNavigation = new MecanumNavigation(this, Configuration.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));

        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));

        robotStateContext.init(); //After mecanum init, because state init could reference mecanumNavigation.
    }

    @Override
    public void loop() {
        super.loop();
        primary.update();
        mecanumNavigation.update();
        robotStateContext.update();
        if (imuUtil != null)
            imuUtil.update();

        mecanumNavigation.displayPosition();

        telemetry.addData("Period Average (sec)", df_precise.format(period.getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_precise.format(period.getMaxPeriodSec()));

        telemetry.addData("State: ", robotStateContext.getCurrentState());
    }

    /**
     * Updates the mecanumNavigation heading from the imu heading.
     * This function forces the IMU to refresh immediately, which
     * causes up to 6ms of latency.
     */
    public void updateMecanumHeadingFromGyroNow() {
        imuUtil.updateNow();
        double gyroHeading = imuUtil.getCompensatedHeading();
        MecanumNavigation.Navigation2D currentPosition = mecanumNavigation.getCurrentPosition();
        currentPosition.theta = Math.toRadians(gyroHeading);
        mecanumNavigation.setCurrentPosition(currentPosition);
    }
}
