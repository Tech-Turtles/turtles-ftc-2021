package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;

@Config
public class Configuration {

    public static double HOPPER_OPEN_POS = 0.32;
    public static double HOPPER_PUSH_POS = 0.22;

    public static final double deadzone = 0.49f;

    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4.0;

    public static final double DRIVE_WHEEL_LATERAL_RATIO = 0.87572 * 120 / 121.47;

    public static final double LAUNCHER_THEORETICAL_MAX = 3387.096774;

    public static final double MM_PER_IN = 25.4f;
    private static double rotationScaleIncrease = 0.99012 * 3600 / 2973.89;
    public static final double WHEELBASE_WIDTH_IN = 16.9 / rotationScaleIncrease;
    public static final double WHEELBASE_LENGTH_IN = 16.0 / rotationScaleIncrease;
    public static final double WHEELBASE_WIDTH_MM = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double WHEELBASE_LENGTH_MM = WHEELBASE_LENGTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES / 2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2.0 *  Math.PI;

    public static final double DRIVE_WHEEL_STEPS_PER_ROT = 28*19.2;
    // Simplify initializing mecanum navigation.
    public static MecanumNavigation.DriveTrainMecanum getDriveTrainMecanum() {
        return new MecanumNavigation.DriveTrainMecanum(
                WHEELBASE_LENGTH_IN, WHEELBASE_WIDTH_IN,
                DRIVE_WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_STEPS_PER_ROT,
                DRIVE_WHEEL_LATERAL_RATIO);
    }
}