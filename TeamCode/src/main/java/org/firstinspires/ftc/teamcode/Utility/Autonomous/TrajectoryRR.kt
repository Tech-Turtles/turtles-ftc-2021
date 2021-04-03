package org.firstinspires.ftc.teamcode.Utility.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount.*
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class TrajectoryRR constructor(sampleMecanumDrive: SampleMecanumDrive){
    val drive: SampleMecanumDrive = sampleMecanumDrive

    /*
   Offset system
    */
    private val offsetWobbleArmReach: Double = 12.0 + 4.0
    private val wobbleDropoffLateral: Double = 3.0
    private val wobblePickupLateral: Double = -3.0
    private val offsetWobbleDropoffAlign = Pose2d(-offsetWobbleArmReach - 6.0, wobbleDropoffLateral, 0.0)
    private val offsetWobbleDropoffDeep = Pose2d(-offsetWobbleArmReach + 2.0, wobbleDropoffLateral, 0.0)
    private val offsetWobbleDropoffShallow = Pose2d(-offsetWobbleArmReach - 2.0, wobbleDropoffLateral, 0.0)
    private val wobbleDropoffRotationRadians: Double = 0.0.toRadians

    private val offsetWobblePickupAlign = Pose2d(-offsetWobbleArmReach - 8.0 + 6.0, wobblePickupLateral, 0.0)
    private val offsetWobblePickupGrab = Pose2d(-offsetWobbleArmReach + 3.0 + 4.0 + 3.0, wobblePickupLateral, 0.0) // Drive through wobble goal
    private val wobblePickupRotationRadians: Double = (165.0).toRadians

    private val offsetRingPickupAlign = Pose2d(-18.0, -5.0, 0.0)
    private val offsetRingPickupGrab = Pose2d(10.0, -5.0, 0.0)
    private val ringPickupRotationRadians: Double = (149.0).toRadians

    private val spacingPowershot: Double = 7.0 // Spacing between the powershot sticks in the y axis, inches

    // old offsets
    val ringOffset: Pose2d = Pose2d(2.0, 5.0, 0.0)
    private val wobbleOffset: Pose2d = Pose2d(-12.0, 0.0, 0.0)
    private val wobblePickup: Pose2d = Pose2d(12.0, 0.0, 0.0)

    // Actual locations before offsets for robot grabber
    private var RINGS_ACTUAL = Pose2d(-24.0, -36.0, Math.toRadians(0.0))  // Needs rotation for pickup
    private var WOBBLE_PICKUP_ACTUAL = Pose2d(-48.0, -50.0, Math.toRadians(0.0)) // Needs rotation for pickup
    private var ZONE_A_CENTER = Pose2d(12.0, -60.0, Math.toRadians(0.0))
    private var ZONE_B_CENTER = Pose2d(36.0, -36.0, Math.toRadians(0.0))
    private var ZONE_C_CENTER = Pose2d(60.0, -60.0, Math.toRadians(0.0))

    // Start Positions
    // True X is -60.75, not -62.0
    //TODO Change start position and update old positions with the new value
    val START_WALL = Pose2d(-62.0, -42.0, Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -18.0, Math.toRadians(180.0 + 0.0))


    var SHOOT_HIGHGOAL      = Pose2d(-4.0, -42.0 + 2.0 + 10.0 - 4.0, Math.toRadians(180.0 - 0.0))
    var POWERSHOT_LEFT      = Pose2d(-4.0, -6.5 + spacingPowershot + -3.0 + 1.0, Math.toRadians(180.0 - 0.0))
    var POWERSHOT_CENTER    = POWERSHOT_LEFT.plus(Pose2d(0.0, -1.0 * spacingPowershot - 1.5 + 2.0 - 1.0 -0.75, 0.0))
    var POWERSHOT_RIGHT     = POWERSHOT_LEFT.plus(Pose2d(0.0, -2.0 * spacingPowershot - 1.5 + 3.0 - 2.5 , 0.0))


    // Offset added to rings
    var RINGS = RINGS_ACTUAL.plus(Pose2d(8.0, 4.0, 0.0)
            .rotateFrame(-45.0.toRadians))//.plus(ringOffset)
    var SHOOT = SHOOT_HIGHGOAL


    // Variable waypoints
    var ZONE_CENTER_VARIABLE = ZONE_A_CENTER
    var wobbleDropoffAlign: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffAlign.rotateFrame(
            wobbleDropoffRotationRadians))
    var wobbleDropoffDeep: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffDeep.rotateFrame(
            wobbleDropoffRotationRadians))
    var wobbleDropoffShallow: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffShallow.rotateFrame(
            wobbleDropoffRotationRadians))
    var wobblePickupAlign: Pose2d = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupAlign.rotateFrame(
            wobblePickupRotationRadians))
    var wobblePickupGrab: Pose2d = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupGrab.rotateFrame(
            wobblePickupRotationRadians))
    var ringPickupAlign: Pose2d = RINGS_ACTUAL.plus(offsetRingPickupAlign.rotateFrame(ringPickupRotationRadians))
    var ringPickupGrab: Pose2d = RINGS_ACTUAL.plus(offsetRingPickupGrab.rotateFrame(ringPickupRotationRadians))


    // Older waypoints
    var CENTER_TO_SHOOT = Pose2d(-2.0, -6.0, 0.0.toRadians)
    var RIGHT_TO_SHOOT = Pose2d(-2.0, -20.0, 0.0.toRadians)
    var POWER_SHOT = Pose2d(-2.0, -28.0, Math.toRadians(180.0 - 0.0))

    var ZONE_A = ZONE_A_CENTER.plus(wobbleOffset)
    var ZONE_B = ZONE_B_CENTER.plus(wobbleOffset)
    var ZONE_C = ZONE_C_CENTER.plus(wobbleOffset)
    var PARK = Pose2d(12.0, -42.0, Math.toRadians(180.0))
    var FAR_PARK = Pose2d(12.0, -15.0, Math.toRadians(180.0))
    var WOBBLE_WALL = Pose2d(-48.0, -50.0, Math.toRadians(180.0)).plus(wobblePickup)


    var WALL_WAY = Pose2d(-24.0, -56.0, Math.toRadians(180.0))
    var WALL_WAY_START = WALL_WAY.plus(Pose2d(-15.0, 4.0, 0.0))
    var CENTER_WAY_START = Pose2d(-36.0, 0.0, 180.0.toRadians)
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    var ZONE_VARIABLE: Pose2d = ZONE_C
    var wobbleTangent: Double = -45.0

    // Define trajectories
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
    var trajCenterStartToPowershotLeft: Trajectory? = null
    var trajPowershotLeftToPowershotCenter: Trajectory? = null
    var trajPowershotCenterPowershotRight: Trajectory? = null
    var trajPowershotRightToWobbleDropOff: Trajectory? = null


    var trajShootToWallWobblePickup: Trajectory? = null
    var trajStartWallToStartCenter: Trajectory? = null
    var trajClaimWobbleToZone: Trajectory? = null
    var trajParkAfterWobbleDropoff: Trajectory? = null
    var trajPickupRingsFromZone: Trajectory? = null

    // New Trajectories
    var trajWobbleDropoffToWobblePickupAlign: Trajectory? = null
    var trajWobbleAlignToWobblePickup: Trajectory? = null
    var trajWobblePickupToDropoffAlign: Trajectory? = null
    var trajWobbleAlignToSecondDropoff: Trajectory? = null
    var trajSecondWobbleDropoffToPark: Trajectory? = null
    var trajSecondWobbleDropoffToRingPickupAlign: Trajectory? = null
    var trajRingAlignToRingGrab: Trajectory? = null
    var trajRingGrabToShootHighGoal: Trajectory? = null
    var trajFromShootHighGoalToPark: Trajectory? = null


    // Pickup rings after powershot but before first wobble pickup
    //trajPowershotRightToWobbleDropoff // Defined above
    var trajPowershotRightToRingPickupAlign: Trajectory? = null

    // 5 trajectories to support high goal and highgoal + ring pickup trajectories
    var trajCenterStartToHighGoal: Trajectory? = null
    var trajHighGoalToRingAlign: Trajectory? = null
    //trajRingAlignToRingGrab already defined (vs new unused name trajRingAlignToRingPickup)
    //trajRingGrabToShootHighGoal already defined (vs new unused name trajRingPickupToHighGoal)
    var trajHighGoalToWobbleDropoffDeep: Trajectory? = null

    var trajCenterStartToPark: Trajectory? = null



    private var velocityConstraint: TrajectoryVelocityConstraint? = null
    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var slowVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var slowAccelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var ringVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var ringAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private val slowVelocity: Double = 10.0
    private val slowAcceleration: Double = 40.0
    private val ringVelocity: Double = 60.0
    private val ringAcceleration: Double = 40.0

    val list = ArrayList<Trajectory>()

    fun setZone(ringAmount: RingDetectionAmount) {
        ZONE_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A
            ONE -> ZONE_B
            FOUR -> ZONE_C
        }
        ZONE_CENTER_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A_CENTER
            ONE -> ZONE_B_CENTER
            FOUR -> ZONE_C_CENTER
        }
        wobbleTangent = when (ringAmount) {
            ZERO -> -40.0
            ONE -> 0.0
            FOUR -> -15.0
        }

        buildTrajectories()
    }


    init {
        velocityConstraint = getMinVelocityConstraint(DriveConstants.MAX_VEL)
        accelerationConstraint = getMinAccelerationConstraint(DriveConstants.MAX_ACCEL)

        slowVelocityConstraint = getMinVelocityConstraint(slowVelocity)
        slowAccelerationConstraint = getMinAccelerationConstraint(slowAcceleration)

        ringVelocityConstraint = getMinVelocityConstraint(ringVelocity)
        ringAccelerationConstraint = getMinAccelerationConstraint(ringAcceleration)

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
        // Redundant because trajectory is split into 4
        val trajPowershot_clockwise: Trajectory =
                trajectoryBuilder(START_CENTER, 90.0.toRadians)
                .splineToConstantHeading(CENTER_TO_SHOOT.vec(), (-90.0).toRadians)
                .lineTo(RIGHT_TO_SHOOT.vec(), slowVelocityConstraint, slowAccelerationConstraint)
                .splineToSplineHeading(ZONE_VARIABLE, Math.toRadians(wobbleTangent), velocityConstraint, accelerationConstraint)
                .build()
        this.trajPowershot_clockwise = trajPowershot_clockwise


        /*
        **************************************************************
        * Rework trajectories
         */

        // Recalculate class variable waypoints based on ZONE_CENTER_VARIABLE
        wobbleDropoffAlign = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffAlign.rotateFrame(wobbleDropoffRotationRadians))
        wobbleDropoffDeep = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffDeep.rotateFrame(wobbleDropoffRotationRadians))
        wobbleDropoffShallow = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffShallow.rotateFrame(wobbleDropoffRotationRadians))
        wobblePickupAlign = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupAlign.rotateFrame(wobblePickupRotationRadians))
        wobblePickupGrab = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupGrab.rotateFrame(wobblePickupRotationRadians))
        ringPickupAlign = RINGS_ACTUAL.plus(offsetRingPickupAlign.rotateFrame(ringPickupRotationRadians))
        ringPickupGrab = RINGS_ACTUAL.plus(offsetRingPickupGrab.rotateFrame(ringPickupRotationRadians))

        /* Split traj_powershot_clockwise into 4 parts
        parkCenterToPsLeft
        PsLeftToPsCenter
        PsCenterToPsRight
        PsRightToWobbleZone
         */

        val traj_CenterStartToPowershotLeft: Trajectory =
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
                        //.splineToConstantHeading(POWERSHOT_LEFT.vec(),45.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_LEFT.vec())
                        .build()
        this.trajCenterStartToPowershotLeft = traj_CenterStartToPowershotLeft


        val traj_PowershotLeftToPowershotCenter: Trajectory =
                trajectoryBuilder(traj_CenterStartToPowershotLeft.end(), -90.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_CENTER.vec())
                        .build()
        this.trajPowershotLeftToPowershotCenter = traj_PowershotLeftToPowershotCenter


        val traj_PowershotCenterPowershotRight: Trajectory =
                trajectoryBuilder(traj_PowershotLeftToPowershotCenter.end(), -90.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_RIGHT.vec())
                        .build()
        this.trajPowershotCenterPowershotRight = traj_PowershotCenterPowershotRight


        /*
             Option: From powershot_right go to ring pickup
         */
        // Powershot Right to Ring Align
        val trajPowershotRightToRingPickupAlign: Trajectory =
                trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), -90.0.toRadians)
                        .lineToLinearHeading(ringPickupAlign)
                        .build()
        this.trajPowershotRightToRingPickupAlign = trajPowershotRightToRingPickupAlign


        /*
            If skipping rings, go straight to wobble dropoff
         */
        val zoneBDeepOffset =  Pose2d(3.0, 3.5 - 1.0, 0.0)
        val traj_PowershotRightToWobbleDropoff: Trajectory =
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), (-90.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0, 6.0, 0.0)), Math.toRadians(-90.0))
                                .lineToConstantHeading(wobbleDropoffDeep.vec())
                                .build()
                    ZONE_B_CENTER ->
                        trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), (-50.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneBDeepOffset), 0.0)
                                .lineToConstantHeading(wobbleDropoffDeep.plus(zoneBDeepOffset).vec())
                                .build()
                    else -> // Zone C
                        trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), 0.0.toRadians)
                                // SIMPLE OPTION - turns wrong way near the wall
                                //.lineToLinearHeading(wobbleDropoffDeep)
                                // FANCY OPTION - turns away from the wall
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(-15.0, 5.0, 1.0.toRadians)), (-20.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(3.0, 0.0 , 0.0)), (-20.0).toRadians)
                                .build()
                }
        this.trajPowershotRightToWobbleDropOff = traj_PowershotRightToWobbleDropoff

        // From zone wobble dropoff position to rings pickup align
        val trajWobbleDropoffToWobblePickupAlign: Trajectory =
                when (ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(traj_PowershotRightToWobbleDropoff.end(), 120.0.toRadians)
                                .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                                .build()
                    ZONE_B_CENTER ->
                        trajectoryBuilder(traj_PowershotRightToWobbleDropoff.end(), 180.0.toRadians)
                                .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                                .build()
                    else ->
                        trajectoryBuilder(traj_PowershotRightToWobbleDropoff.end(), 150.0.toRadians)
                                .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                                .build()
                }
        this.trajWobbleDropoffToWobblePickupAlign = trajWobbleDropoffToWobblePickupAlign


        // Grab Wobble Goal
        val trajWobbleAlignToWobblePickup: Trajectory =
                trajectoryBuilder(trajWobbleDropoffToWobblePickupAlign.end(), 0.0.toRadians)
                        .lineToConstantHeading(wobblePickupGrab.vec(), slowVelocityConstraint, slowAccelerationConstraint)
                        .build()
        this.trajWobbleAlignToWobblePickup = trajWobbleAlignToWobblePickup

        // Align to dropoff second wobble goal
        val zoneBShallowOffset = Pose2d(-5.0, -2.0, 0.0)
        val trajWobblePickupToDropoffAlign: Trajectory =
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                                .splineToSplineHeading(wobbleDropoffAlign, Math.toRadians(-30.0))
                                //.lineToConstantHeading(wobbleDropoffAlign.vec())
                                .build()
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                                .lineToConstantHeading(wobblePickupAlign.plus(zoneBShallowOffset).vec())
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneBShallowOffset), 25.0.toRadians)
                                //.lineToConstantHeading(wobbleDropoffDeep.vec())
                                .build()
                    else -> // Zone C
                        trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + (180.0).toRadians)
                                //.lineToConstantHeading(wobblePickupAlign.vec())
                                .splineToSplineHeading(wobbleDropoffAlign, -20.0.toRadians)
                                .build()
                }
        this.trajWobblePickupToDropoffAlign = trajWobblePickupToDropoffAlign

        // Dropoff Second WobbleGoal
        val trajWobbleAlignToSecondDropoff: Trajectory =
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajWobblePickupToDropoffAlign.end(), 0.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffShallow.plus(zoneBShallowOffset).vec())
                                .build()
                    else -> // Zone C
                        trajectoryBuilder(trajWobblePickupToDropoffAlign.end(), 0.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffShallow.vec())
                                .build()
                }
        this.trajWobbleAlignToSecondDropoff = trajWobbleAlignToSecondDropoff


        // Second WobbleGoal to Park (safe option)
        val trajSecondWobbleDropoffToPark: Trajectory =
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffAlign.vec())
                                .splineToConstantHeading(PARK.vec().plus(Vector2d(0.0, 12.0)), 0.0.toRadians)
                                .build()
                    else ->
                        trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 135.0.toRadians)
                                .lineToConstantHeading(PARK.vec().plus(Vector2d(-8.0, 12.0)))
                                .build()
                }
        this.trajSecondWobbleDropoffToPark = trajSecondWobbleDropoffToPark


        // Second WobbleGoal to Ring Pickup Align
        val trajSecondWobbleDropoffToRingPickupAlign: Trajectory =
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffAlign.vec().plus(Vector2d(0.0, 7.0)))
                                .splineToSplineHeading(ringPickupAlign, 90.0.toRadians + 0.0 * ringPickupRotationRadians)
                                //.lineToLinearHeading( ringPickupAlign)
                                .build()
                    else ->
                        trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                                .lineToLinearHeading(ringPickupAlign)
                                .build()
                }
        this.trajSecondWobbleDropoffToRingPickupAlign = trajSecondWobbleDropoffToRingPickupAlign


        // Pickup Rings
        val trajRingAlignToRingGrab: Trajectory =
                trajectoryBuilder(ringPickupAlign, 0.0.toRadians)
                        .lineToConstantHeading(ringPickupGrab.vec(), ringVelocityConstraint, ringAccelerationConstraint)
                        .build()
        this.trajRingAlignToRingGrab = trajRingAlignToRingGrab


        // Take picked up rings to shoot
        val trajRingGrabToShootHighGoal: Trajectory =
                trajectoryBuilder(ringPickupGrab, 0.0.toRadians)
                        .lineToLinearHeading(SHOOT_HIGHGOAL)
                        .build()
        this.trajRingGrabToShootHighGoal = trajRingGrabToShootHighGoal


        // After shooting into highgoal, go park
        val trajFromShootHighGoalToPark: Trajectory =
                trajectoryBuilder(SHOOT_HIGHGOAL, 0.0.toRadians)
                        .lineToLinearHeading(PARK)
                        .build()
        this.trajFromShootHighGoalToPark = trajFromShootHighGoalToPark



        //
        // End New stuff 2/6/2021


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
        val trajParkAfterWobbleDropoff: Trajectory =
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



        /*
           5 new trajectories for doing high shot and optionally rings pickup
            trajCenterStartToHighGoal
            trajHighGoalToRingAlign
            trajRingAlignToRingGrab // Already defined
            trajRingGrabToShootHighGoal // Already defined
            trajHighGoalToWobbleDropoffDeep
         */

        // Start Center to HighGoal shoot position
        // Calculate x position for smooth turn around rings
        val xPositionWeighted = (2.0 * RINGS_ACTUAL.x + 1.0 * SHOOT_HIGHGOAL.x) / 3.0
        val ringLeftToHighGoal = Pose2d(xPositionWeighted, START_CENTER.y, 0.0.toRadians);
        val trajCenterStartToHighGoal: Trajectory =
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
                        .splineTo(ringLeftToHighGoal.vec(), 0.0.toRadians)
                        .splineToLinearHeading(SHOOT_HIGHGOAL, -80.0.toRadians) // Approach direction
                        //.splineTo(ringLeftToHighGoal)
                        //.(SHOOT_HIGHGOAL)
                        //.lineToLinearHeading(SHOOT_HIGHGOAL)
                        .build()
        this.trajCenterStartToHighGoal = trajCenterStartToHighGoal


        // High Goal to Ring Align
        val trajHighGoalToRingAlign: Trajectory =
                trajectoryBuilder(SHOOT_HIGHGOAL, 180.0.toRadians)
                        .lineToLinearHeading(ringPickupAlign)
                        .build();
        this.trajHighGoalToRingAlign = trajHighGoalToRingAlign


        // RingAlign to  Ring Pickup     --- OLD TRAJECTORY
        // trajRingAlignToRingGrab

        // Ring pickup to high goal     --- OLD TRAJECTORY
        // trajRingGrabToShootHighGoal


        // High Goal to Wobble Drop (NOT created elsewhere)
        val trajHighGoalToWobbleDropoffDeep: Trajectory =
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajRingGrabToShootHighGoal.end(), -60.0.toRadians)
                                // SIMPLE OPTION - turns wrong way near the wall
                                //.splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0,6.0,0.0)),Math.toRadians(-90.0))
                                //.lineToConstantHeading(wobbleDropoffDeep.vec())
                                // FANCY OPTION - turns away from the wall
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0, 6.0, 1.0.toRadians)), Math.toRadians(-90.0))
                                .lineToSplineHeading(wobbleDropoffDeep)
                                .build();
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajRingGrabToShootHighGoal.end(), 30.0.toRadians)
                                //.lineToLinearHeading(wobbleDropoffDeep)
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneBDeepOffset), 0.0)
                                .lineToConstantHeading(wobbleDropoffDeep.plus(zoneBDeepOffset).vec())
                                .build();
                    else -> // Zone C
                        trajectoryBuilder(trajRingGrabToShootHighGoal.end(), -20.0.toRadians)
                                // SIMPLE OPTION - turns wrong way near the wall
                                //.lineToLinearHeading(wobbleDropoffDeep)
                                // FANCY OPTION - turns away from the wall
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(-20.0, 3.0, 1.0.toRadians)), -20.0.toRadians)
                                .splineToSplineHeading(wobbleDropoffDeep, 0.0.toRadians)
                                .build();
                }
        this.trajHighGoalToWobbleDropoffDeep = trajHighGoalToWobbleDropoffDeep

        val trajCenterStartToPark: Trajectory =
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
                        //.splineToConstantHeading(POWERSHOT_LEFT.vec(),45.0.toRadians)
                        .lineToConstantHeading(FAR_PARK.vec())
                        .build()
        this.trajCenterStartToPark = trajCenterStartToPark
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
            val flipOffset = Pose2d(0.75, 0.0, 180.0.toRadians)
            val corners = ArrayList<Pose2d>()
            corners.add(Pose2d(-61.5, -61.0, (0.0).toRadians))
            // TODO: re-enable other 3 corners
//            corners.add(Pose2d( 60.75, -61.0, (0.0).toRadians))
//            corners.add(Pose2d(-61.5, 61.0, (0.0).toRadians))
//            corners.add(Pose2d( 60.75, 61.0, (0.0).toRadians))

            // Which direction, fwd 0.0 or reverse 180.0?
            // TODO: re-enable forward orientation
            val isHeadingFwd = false // HARDCODED
            //val isHeadingFwd = Math.abs(0.0.toRadians - pose.heading) < 90.0.toRadians

            val orientationOffset =
                if(isHeadingFwd) Pose2d(0.0, 0.0, 0.0)
                else flipOffset

            // Which point is the closest?
            var closestCorner = corners[0].plus(orientationOffset)
            var closestDistance = getDistance(pose, closestCorner)
            for(corner in corners) {
                var newCorner = corner
                var newDistance = getDistance(pose, newCorner)
                if(newDistance < closestDistance) {
                    closestCorner = newCorner.plus(orientationOffset)
                    closestDistance = newDistance
                }
            }
            return closestCorner
        }

        fun getDistance(pose1: Pose2d, pose2: Pose2d): Double {
            val deltaX = abs(pose1.x - pose2.x)
            val deltaY = abs(pose1.y - pose2.y)
            return sqrt(deltaX.pow(2.0) + deltaY.pow(2.0))
        }
    }

    fun getMinVelocityConstraint(MaxVelocity: Double): MinVelocityConstraint {
        return MinVelocityConstraint(listOf(
                AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                MecanumVelocityConstraint(MaxVelocity, DriveConstants.TRACK_WIDTH)
        ))
    }

    fun getMinAccelerationConstraint(MaxAccel: Double): ProfileAccelerationConstraint {
        return ProfileAccelerationConstraint(MaxAccel)
    }
}

val Double.toRadians get() = (Math.toRadians(this))

/*
    +x is the 'positive' direction, and rotation is counter-clockwise around (0,0)
    https://en.wikipedia.org/wiki/Rotation_matrix
 */
fun Pose2d.rotateFrame(rotationRadians: Double): Pose2d
{
    return Pose2d(this.x * kotlin.math.cos(rotationRadians) - this.y * kotlin.math.sin(rotationRadians),
            this.x * kotlin.math.sin(rotationRadians) + this.y * kotlin.math.cos(rotationRadians),
            this.heading + rotationRadians)
}