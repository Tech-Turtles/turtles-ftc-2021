package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

public class TrajectoryRR {

    public SampleMecanumDrive drive;

    Pose2d START_WALL = new Pose2d(-62,-42,Math.toRadians(180));
    Pose2d START_CENTER = new Pose2d(-62,-24,Math.toRadians(180));
    Pose2d RINGS = new Pose2d(-24,-36,Math.toRadians(180));
    Pose2d SHOOT = new Pose2d(-2,-36,Math.toRadians(180));
    Pose2d ZONE_A = new Pose2d(12,-60,Math.toRadians(0)); // could be -90 so overshooting hits the wall
    Pose2d ZONE_B = new Pose2d(-36,36,Math.toRadians(0));
    Pose2d ZONE_C = new Pose2d(-60,60,Math.toRadians(0));
    Pose2d PARK = new Pose2d(12,-36,Math.toRadians(180));

    Pose2d TO_ZONE = new Pose2d(-24,-60,Math.toRadians(180));
    Pose2d ZONE_VARIABLE;

    public Trajectory Traj_shoot;
    public Trajectory Traj_pickup_rings;
    public Trajectory Traj_shoot2;
    public Trajectory Traj_park;

    public TrajectoryRR(SampleMecanumDrive drive) {
        this.drive = drive;
        setZone(Zones.A);
        init();
    }

    enum Zones {
        A,B,C,
    }

    public void setZone(Zones zone) {
        switch(zone) {
            case A:
                ZONE_VARIABLE = ZONE_A;
                break;
            case B:
                ZONE_VARIABLE = ZONE_B;
                break;
            case C:
                ZONE_VARIABLE = ZONE_C;
                break;
            default:
        }

        init();
    }


    public void init() {
        /*
        2 starting positions
        3 wobble goal positions
        also....
        different routines to work up to
        A) drive,shoot, park
        B) drive,shoot, pickup rings, park
        C) drive, drop wobble, shoot, pickup rings, shoot 2 park
        D) drive, drop wobble, shoot, pickup rings, shoot 2, second wobble, park
         */

        Traj_shoot = drive.trajectoryBuilder(START_WALL)
                .lineToLinearHeading(TO_ZONE)
                //.splineTo(ZONE_VARIABLE)
                .lineToLinearHeading(SHOOT)
                .build();

        Traj_pickup_rings = drive.trajectoryBuilder(SHOOT)
                .lineToLinearHeading(RINGS)
                .build();

        Traj_shoot2 = drive.trajectoryBuilder(RINGS)
                .lineToLinearHeading(SHOOT)
                .build();

        Traj_park = drive.trajectoryBuilder(SHOOT)
                .lineToLinearHeading(PARK)
                .build();
    }
}
