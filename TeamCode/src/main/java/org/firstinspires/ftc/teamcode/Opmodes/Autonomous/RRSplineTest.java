package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@Autonomous
public class RRSplineTest extends RobotHardware {

    SampleMecanumDrive drive;
    Trajectory trajectory;

    @Override
    public void init() {
        super.init();
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        drive.setPoseEstimate(new Pose2d(-62.0, -42.0, Math.toRadians(180)));
        trajectory = drive.trajectoryBuilder(new Pose2d(-62.0, -42.0), Math.toRadians(180))
                .splineTo(new Vector2d(-52.0, -42.0), 0)
                .splineTo(new Vector2d(-22.0, -59.0), 0)
                .splineTo(new Vector2d(-0.0, -34.0), 0)
                .build();
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
