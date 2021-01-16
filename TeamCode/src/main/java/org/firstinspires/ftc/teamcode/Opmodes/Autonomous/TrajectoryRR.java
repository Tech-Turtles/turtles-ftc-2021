package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount;

public class TrajectoryRR {

    public SampleMecanumDrive drive;


    Pose2d ringOffset = new Pose2d(2.0,5.0,0.0);
    Pose2d wobbleOffset = new Pose2d(-12.0,0.0,0.0);

    Pose2d START_WALL = new Pose2d(-62.0, -42.0,Math.toRadians(180.0));
    Pose2d START_CENTER = new Pose2d(-62.0, -24.0,Math.toRadians(180.0));
    Pose2d RINGS = new Pose2d(-24.0, -36.0,Math.toRadians(180.0)).plus(ringOffset);
    Pose2d SHOOT = new Pose2d(-2.0, -42.0,Math.toRadians(180.0));
    Pose2d ZONE_A = new Pose2d(12.0, -60.0,Math.toRadians(0.0)).plus(wobbleOffset);
    Pose2d ZONE_B = new Pose2d(36.0, -36.0,Math.toRadians(0.0)).plus(wobbleOffset);
    Pose2d ZONE_C = new Pose2d(60.0, -60.0,Math.toRadians(0.0)).plus(wobbleOffset);
    Pose2d PARK = new Pose2d(12.0, -42.0,Math.toRadians(180.0));

    // TO_ZONE renamed to WALL_WAY
    Pose2d WALL_WAY = new Pose2d(-24.0, -56.0,Math.toRadians(180.0));
    Pose2d WALL_WAY_START = WALL_WAY.plus(new Pose2d(-15.0,4.0,0.0));
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    Pose2d ZONE_VARIABLE = ZONE_A;
    Double wobbleTangent = -45.0;


    public Trajectory trajToShoot1;
    public Trajectory trajToPark;
    public Trajectory trajPickupRings;
    public Trajectory trajToShoot2;
    public Trajectory trajStartToZone;
    public Trajectory trajFromShootToZone;
    public Trajectory trajZoneToShoot1;


    public TrajectoryRR(SampleMecanumDrive drive) {
        this.drive = drive;
        setZone(RingDetectionAmount.ZERO);
        init();
    }

    public void setZone(RingDetectionAmount amount) {
        switch(amount) {
            case ZERO:
                ZONE_VARIABLE = ZONE_A;
                wobbleTangent = -45.0;
                break;
            case ONE:
                ZONE_VARIABLE = ZONE_B;
                wobbleTangent = 0.0;
                break;
            case FOUR:
                ZONE_VARIABLE = ZONE_C;
                wobbleTangent = -45.0;
                break;
            default:
        }

        init();
    }

    public Vector2d toVector2d(Pose2d pose) {
        return new Vector2d(pose.getX(),pose.getY());
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


        // Start with diagonal 2.5
        trajToShoot1 = drive.trajectoryBuilder(START_WALL)
                        //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                        .lineTo(toVector2d(WALL_WAY_START))
                        .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
                        .splineToLinearHeading(SHOOT,Math.toRadians(90.0))
                        .build();

        // Park immediately after shooting
        trajToPark = drive.trajectoryBuilder(trajToShoot1.end(),Math.toRadians(0.0))
                        .splineToSplineHeading(PARK,Math.toRadians(0.0))
                        .build();
        //list.add(trajToPark)

        // From shooting position to rings pickup
        trajPickupRings = drive.trajectoryBuilder(trajToShoot1.end())
                    //.splineToSplineHeading(RINGS,Math.toRadians(0.0))
                    .splineTo(toVector2d(RINGS),Math.toRadians(180.0 - 45.0))
                .build();

        // Second batch of shooting after picking up rings
        // Fix heading term? Set to 0.0 or 180.0 ?
        trajToShoot2 = drive.trajectoryBuilder(trajPickupRings.end(), trajPickupRings.end().getHeading() + Math.toRadians(180.0))
                    //.splineToSplineHeading(SHOOT,Math.toRadians(0.0))
                .splineToLinearHeading(SHOOT,Math.toRadians(0.0))
                .build();

        // Drive from start to Zone
        trajStartToZone = drive.trajectoryBuilder(START_WALL)
                        //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                        .lineTo(toVector2d(WALL_WAY_START))
                        .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
                        .splineToSplineHeading(ZONE_VARIABLE,Math.toRadians(wobbleTangent))
                        .build();

        // Drive from shoot to Zone
        trajFromShootToZone = drive.trajectoryBuilder(SHOOT)
                        //.splineTo(toVector2d(ZONE_VARIABLE),Math.toRadians(wobbleTangent))
                        .lineToLinearHeading(ZONE_VARIABLE)
                        .build();

        // Drive from Zone to Shoot1
        trajZoneToShoot1 = drive.trajectoryBuilder(trajStartToZone.end())
                        .lineToLinearHeading(SHOOT)
                        .build();
    }
}
