package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.BehaviorSandBox;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.AutoDrive;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;


public class AutoOpmode extends RobotHardware {

    AllianceColor robotColor;
    StartPosition robotStartPos;
    private Executive.RobotStateMachineContextInterface robotStateContext;

    @Autonomous(name="Wall Red", group="A")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.WALL;
            super.init();
        }
    }

    @Autonomous(name="Center Red", group="A")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.CENTER;
            super.init();
        }
    }

    @Autonomous(name="Wall Blue", group="A")
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.WALL;
            super.init();
        }
    }

    @Autonomous(name="Center Blue", group="A")
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.CENTER;
            super.init();
        }
    }

    @Autonomous(name="Sandbox", group="E")
    public static class Sandbox extends AutoOpmode {}

    @Override
    public void init() {
        super.init();
        imuUtil = new IMUUtilities(this, IMU.IMU1.getName());
        if(robotColor == null && robotStartPos == null)
            robotStateContext = new BehaviorSandBox(AutoOpmode.this, AllianceColor.RED, StartPosition.CENTER);
        else
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

        autoDrive = new AutoDrive(this, mecanumNavigation, mecanumNavigation);

        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));

        robotStateContext.init(); //After mecanum init, because state init could reference mecanumNavigation.
    }

    @Override
    public void loop() {
        super.loop();

        mecanumNavigation.update();
        robotStateContext.update();

        if (imuUtil != null)
            imuUtil.update();

        mecanumNavigation.displayPosition();

        telemetry.addData("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
        telemetry.addData("Period Max:     ", df_precise.format(period.getMaxPeriodSec()) + "s");
        telemetry.addData("State:          ", robotStateContext.getCurrentState());
    }
}