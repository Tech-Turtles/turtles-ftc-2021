package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@Autonomous(name = "Trajectory Tester", group = "E")
public class TrajectoryTester extends RobotHardware {
    TrajectoryRR trajectoryRR = null;
    Pose2d startPosition = new Pose2d();
    Trajectory routine;

    @Override
    public void init() {
        super.init();
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));
        trajectoryRR = new TrajectoryRR(mecanumDrive);
        startPosition = trajectoryRR.getSTART_CENTER();
        mecanumDrive.setPoseEstimate(startPosition);
        routine = trajectoryRR.getTrajCenterStartToPowershotLeft();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive.followTrajectoryAsync(routine);
    }

    @Override
    public void loop() {
        super.loop();
        mecanumDrive.update(packet);
    }
}