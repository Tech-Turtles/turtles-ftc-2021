package org.firstinspires.ftc.teamcode.Utility.Autonomous

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount.*
import kotlin.collections.ArrayList


@Config
class TrajectoryRR_kotlin constructor(sampleMecanumDrive: SampleMecanumDrive){
    val drive: SampleMecanumDrive = sampleMecanumDrive
    val ringOffset: Pose2d = Pose2d(2.0, 5.0, 0.0)
    val wobbleOffset: Pose2d = Pose2d(-12.0, 0.0, 0.0)
    val wobblePickup: Pose2d = Pose2d(12.0, 0.0, 0.0)
    val spacing_powershot: Double = 4.0;

    val START_WALL = Pose2d(-62.0, -42.0, Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -18.0, Math.toRadians(180.0))
    var RINGS = Pose2d(-24.0, -36.0, Math.toRadians(180.0)).plus(ringOffset)
    var SHOOT = Pose2d(-2.0, -42.0 + 2.0, Math.toRadians(180.0 - 0.0))
    var CENTER_TO_SHOOT = Pose2d(-2.0, -6.0, 0.0.toRadians)
    var RIGHT_TO_SHOOT = Pose2d(-2.0, -20.0, 0.0.toRadians)
    var POWER_SHOT = Pose2d(-2.0, -28.0, Math.toRadians(180.0 - 0.0))
    var POWERSHOT_CENTER = Pose2d(-2.0,  -12.0,Math.toRadians(180.0 - 0.0))
    var POWERSHOT_LEFT  = POWERSHOT_CENTER.plus(Pose2d(0.0,+1.0 * spacing_powershot, 0.0))
    var POWERSHOT_RIGHT = POWERSHOT_CENTER.plus(Pose2d(0.0,-1.0 * spacing_powershot, 0.0))
    var ZONE_A = Pose2d(12.0, -60.0, Math.toRadians(0.0)).plus(wobbleOffset)
    var ZONE_B = Pose2d(36.0, -36.0, Math.toRadians(0.0)).plus(wobbleOffset)
    var ZONE_C = Pose2d(60.0, -60.0, Math.toRadians(0.0)).plus(wobbleOffset)
    var PARK = Pose2d(12.0, -42.0, Math.toRadians(180.0))
    var WOBBLE_WALL = Pose2d(-48.0, -50.0, Math.toRadians(180.0)).plus(wobblePickup)


    var WALL_WAY = Pose2d(-24.0, -56.0, Math.toRadians(180.0))
    var WALL_WAY_START = WALL_WAY.plus(Pose2d(-15.0, 4.0, 0.0))
    var CENTER_WAY_START = Pose2d(-36.0, 0.0, 180.0.toRadians)
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    var ZONE_VARIABLE: Pose2d = ZONE_C
    var wobbleTangent: Double = -45.0

    var trajToShoot1: Trajectory? = null
    var trajToPark: Trajectory? = null
    var trajPickupRings: Trajectory? = null
    var trajToShoot2: Trajectory? = null
    var trajStartToZone: Trajectory? = null
    var trajFromShootToZone: Trajectory? = null
    var trajZoneToShoot1: Trajectory? = null
    var trajToPOWERSHOT: Trajectory? = null
    var trajPowershot_clockwise: Trajectory? = null
    // Split clockwise into 4 parts
    var traj_parkCenterToPowershotLeft: Trajectory? = null
    var traj_PowershotLeftToPowershotCenter: Trajectory? = null
    var traj_PowershotCenterPowershotRight: Trajectory? = null
    var traj_PowershotRightToWobbleDropoff: Trajectory? = null


    var trajShootToWallWobblePickup: Trajectory? = null
    var trajStartWallToStartCenter: Trajectory? = null
    var trajClaimWobbleToZone: Trajectory? = null
    var trajParkAfterWobbleDropoff: Trajectory? = null
    var trajPickupRingsFromZone: Trajectory? = null

    private var velocityConstraint: TrajectoryVelocityConstraint? = null
    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var slowVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var slowAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private val slowVelocity: Double = 10.0
    private val slowAcceleration: Double = 10.0

    val list = ArrayList<Trajectory>()

    fun setZone(ringAmount: RingDetectionAmount) {
        ZONE_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A
            ONE -> ZONE_B
            FOUR -> ZONE_C
        }
        wobbleTangent = when (ringAmount) {
            ZERO -> -40.0
            ONE -> 0.0
            FOUR -> -15.0
        }

        buildTrajectories()
    }


    init {
        velocityConstraint = getMinVelocityConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
        accelerationConstraint = getMinAccelerationConstraint(DriveConstants.MAX_ACCEL)

        slowVelocityConstraint = getMinVelocityConstraint(DriveConstants.MAX_ANG_VEL, slowVelocity, DriveConstants.TRACK_WIDTH)
        slowAccelerationConstraint = getMinAccelerationConstraint(slowAcceleration)
        buildTrajectories()
        setZone(ZERO)
    }

    private fun buildTrajectories() {


        // Start with diagonal 2.5
        val trajToShoot1: Trajectory =
                trajectoryBuilder(START_WALL, START_WALL.heading)
                        //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                        .lineTo(toVector2d(WALL_WAY_START))
                        .splineToLinearHeading(WALL_WAY, Math.toRadians(0.0))
                        .splineToLinearHeading(SHOOT, Math.toRadians(90.0))
                        //.lineTo(toVector2d(SHOOT))
                        .build()
        list.add(trajToShoot1)
        this.trajToShoot1 = trajToShoot1

        // Powershot!!!
        val trajToPOWERSHOT: Trajectory =
                trajectoryBuilder(START_WALL, START_WALL.heading)
                        //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                        .lineTo(toVector2d(WALL_WAY_START))
                        .splineToLinearHeading(WALL_WAY, Math.toRadians(0.0))
                        .splineToLinearHeading(POWER_SHOT.plus(Pose2d(0.0, 24.0, 0.0)), Math.toRadians(90.0))
                        //.lineTo(toVector2d(SHOOT))
                        .build()
        list.add(trajToPOWERSHOT)
        this.trajToPOWERSHOT = trajToPOWERSHOT

        // Park immediately after shooting
        val trajToPark: Trajectory =
                trajectoryBuilder(trajToShoot1.end(), trajToShoot1.end().heading * 0.0)
                        .splineToSplineHeading(PARK, Math.toRadians(0.0))
                        .build()
        list.add(trajToPark)
        this.trajToPark = trajToPark

        // From shooting position to rings pickup
        val trajPickupRings: Trajectory =
                trajectoryBuilder(trajToShoot1.end(), trajToShoot1.end().heading)
                        //.splineToSplineHeading(RINGS,Math.toRadians(180.0))
                        //.splineToLinearHeading(RINGS,Math.toRadians(180.0))
                        .splineTo(toVector2d(RINGS), Math.toRadians(180.0 - 45.0))
                        .build()
        list.add(trajPickupRings)
        this.trajPickupRings = trajPickupRings

        // Second batch of shooting after picking up rings
        val trajToShoot2: Trajectory =
                trajectoryBuilder(trajPickupRings.end(), trajPickupRings.end().heading + Math.toRadians(180.0))
                        //.splineToSplineHeading(SHOOT,Math.toRadians(0.0))
                        .splineToLinearHeading(SHOOT, Math.toRadians(0.0))
                        .build()
        list.add(trajToShoot2)
        this.trajToShoot2 = trajToShoot2

        // Drive from start to Zone
        val trajStartToZone: Trajectory =
                trajectoryBuilder(START_WALL, START_WALL.heading)
                        //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                        .lineTo(toVector2d(WALL_WAY_START))
                        .splineToLinearHeading(WALL_WAY, Math.toRadians(0.0))
                        .splineToSplineHeading(ZONE_VARIABLE, Math.toRadians(wobbleTangent))
                        .build()
        //list.add(trajStartToZone)
        this.trajStartToZone = trajStartToZone

        // Drive from shoot to Zone
        val trajFromShootToZone: Trajectory =
                trajectoryBuilder(SHOOT, SHOOT.heading)
                        //.splineTo(toVector2d(ZONE_VARIABLE),Math.toRadians(wobbleTangent))
                        .lineToLinearHeading(ZONE_VARIABLE)
                        .build()
        list.add(trajFromShootToZone)
        this.trajFromShootToZone = trajFromShootToZone

        // Drive from Zone to Shoot1
        val trajZoneToShoot1: Trajectory =
                trajectoryBuilder(trajStartToZone.end(), trajStartToZone.end().heading)
                        .lineToLinearHeading(SHOOT)
                        .build()
        list.add(trajZoneToShoot1)
        this.trajZoneToShoot1 = trajZoneToShoot1


        // Clockwise powershot tour
        // Demo relative start position placement
        val trajStartwalltostartcenter: Trajectory =
                trajectoryBuilder(START_WALL, (90.0 - 20.0).toRadians)
                        .splineToConstantHeading(START_CENTER.vec(), (20.0 + 90.0).toRadians)
                        .build();
        this.trajStartWallToStartCenter = trajStartwalltostartcenter

//        setZone(ONE)
        val trajPowershot_clockwise: Trajectory =
                trajectoryBuilder(START_CENTER, 90.0.toRadians)
                .splineToConstantHeading(CENTER_TO_SHOOT.vec(), (-90.0).toRadians)
                .lineTo(RIGHT_TO_SHOOT.vec(), slowVelocityConstraint, slowAccelerationConstraint)
                .splineToSplineHeading(ZONE_VARIABLE, Math.toRadians(wobbleTangent), velocityConstraint, accelerationConstraint)
                .build()
        this.trajPowershot_clockwise = trajPowershot_clockwise
/* Split traj_powershot_clockwise into 4 parts
       parkCenterToPsLeft
       PsLeftToPsCenter
       PsCenterToPsRight
       PsRightToWobbleZone
        */

        var traj_parkCenterToPowershotLeft: Trajectory =
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
                        //.splineToConstantHeading(POWERSHOT_LEFT.vec(),45.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_LEFT.vec())
                        .build();
        this.traj_parkCenterToPowershotLeft = traj_parkCenterToPowershotLeft


        var traj_PowershotLeftToPowershotCenter: Trajectory =
                trajectoryBuilder(traj_parkCenterToPowershotLeft.end(), -90.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_CENTER.vec())
                        .build();
        this.traj_PowershotLeftToPowershotCenter = traj_PowershotLeftToPowershotCenter


        var traj_PowershotCenterPowershotRight: Trajectory =
                trajectoryBuilder(traj_PowershotLeftToPowershotCenter.end(), -90.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_RIGHT.vec())
                        .build();
        this.traj_PowershotCenterPowershotRight = traj_PowershotCenterPowershotRight


        var traj_PowershotRightToWobbleDropoff: Trajectory =
                trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), -90.0.toRadians)
                        //.lineTo(RIGHT_TO_SHOOT.vec())
                        .splineToSplineHeading(ZONE_VARIABLE,Math.toRadians(wobbleTangent))
                        .build();
        this.traj_PowershotRightToWobbleDropoff = traj_PowershotRightToWobbleDropoff



        //
        // From zone wobble drop off position to rings pickup
        val trajPickupRingsFromZone: Trajectory =
                trajectoryBuilder(trajPowershot_clockwise.end(), (wobbleTangent + 180.0).toRadians)
                        .splineToSplineHeading(SHOOT, 180.0.toRadians)
                        .splineTo(toVector2d(RINGS), Math.toRadians(180.0 - 45.0))
                        .build()
        this.trajPickupRingsFromZone = trajPickupRingsFromZone

        // Claim wall wobble goal
        val trajShootToWallWobblePickup: Trajectory =
                trajectoryBuilder(trajZoneToShoot1.end(), trajZoneToShoot1.end().heading)
                        .splineToLinearHeading(WALL_WAY, -180.0.toRadians)
                        //.splineTo(WALL_WAY_START.vec(),-180.0.toRadians)
                        .splineToLinearHeading(WOBBLE_WALL, (-180.0 - 0.0 * 45.0).toRadians)
                        .build()
        this.trajShootToWallWobblePickup = trajShootToWallWobblePickup

        // From Claim Wobble to Zone
        val trajClaimWobbleToZone: Trajectory =
                trajectoryBuilder(trajShootToWallWobblePickup.end(), (-20.0).toRadians)
                        //.splineToLinearHeading(SHOOT, 0.0)
                        .splineToSplineHeading(ZONE_VARIABLE, wobbleTangent.toRadians)
                        .build()
        this.trajClaimWobbleToZone = trajClaimWobbleToZone

        // Park after wobble zone dropoff
        // Note variable trajectory construction using when to prevent knocking wobble goal from ZONE A.
        var trajParkAfterWobbleDropoff: Trajectory =
                when (ZONE_VARIABLE) {
                    ZONE_A ->
                        trajectoryBuilder(trajClaimWobbleToZone.end(), trajClaimWobbleToZone.end().heading + 180.0.toRadians)
                                .splineToConstantHeading(SHOOT.vec(), 0.0.toRadians)
                                .splineTo(PARK.vec(), 0.0)
                                .build()
                    else ->
                        trajectoryBuilder(trajClaimWobbleToZone.end(), trajClaimWobbleToZone.end().heading + 180.0.toRadians)
                                .lineTo(PARK.vec())
                                .build()
                }
        this.trajParkAfterWobbleDropoff = trajParkAfterWobbleDropoff
    }

    fun toVector2d(pose: Pose2d): Vector2d {
        return Vector2d(pose.x, pose.y)
    }

    fun  trajectoryBuilder(pose: Pose2d, heading: Double): TrajectoryBuilder{
        return drive.trajectoryBuilder(pose, heading)
    }

    fun  trajectoryBuilder(pose: Pose2d, reversed: Boolean): TrajectoryBuilder{
        return drive.trajectoryBuilder(pose, reversed)
    }

    companion object {
        @JvmStatic
        fun getNearestCornerPose2d(pose: Pose2d): Pose2d {
            val flipOffset = Pose2d(0.75,0.0,180.0.toRadians)
            val corners = ArrayList<Pose2d>()
            corners.add(Pose2d(-61.5,-61.0, (0.0).toRadians))
            corners.add(Pose2d(60.75,-61.0, (0.0).toRadians))
            corners.add(Pose2d(-61.5,+61.0, (0.0).toRadians))
            corners.add(Pose2d(60.75,+61.0, (0.0).toRadians))

            // Which direction, fwd 0.0 or reverse 180.0?
            val isHeadingFwd = Math.abs(0.0.toRadians - pose.heading) < 90.0.toRadians
            val orientationOffset =
                if(isHeadingFwd) Pose2d(0.0,0.0,0.0)
                else flipOffset

            // Which point is the closest?
            var closestCorner = corners.get(0).plus(orientationOffset)
            var closestDistance = getDistance(pose, closestCorner)
            for(corner in corners) {
                var newCorner = corner;
                var newDistance = getDistance(pose, newCorner)
                if(newDistance < closestDistance) {
                    closestCorner = newCorner.plus(orientationOffset)
                    closestDistance = newDistance
                }
            }
            return closestCorner
        }

        fun getDistance(pose1: Pose2d, pose2: Pose2d): Double {
            val deltaX = Math.abs(pose1.x - pose2.x)
            val deltaY = Math.abs(pose1.y - pose2.y)
            return Math.sqrt( Math.pow(deltaX,2.0) + Math.pow(deltaY,2.0))
        }
    }

    fun getMinVelocityConstraint(maxAngle: Double, MaxVelocity: Double, TrackWidth: Double): MinVelocityConstraint {
        return MinVelocityConstraint(listOf(
                AngularVelocityConstraint(maxAngle),
                MecanumVelocityConstraint(MaxVelocity, TrackWidth)
        ))
    }

    fun getMinAccelerationConstraint(MaxAccel: Double): ProfileAccelerationConstraint {
        return ProfileAccelerationConstraint(MaxAccel)
    }
}

val Double.toRadians get() = (Math.toRadians(this))