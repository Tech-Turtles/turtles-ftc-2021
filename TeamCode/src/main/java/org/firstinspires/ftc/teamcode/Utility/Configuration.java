package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;

@Config
public class Configuration {

    public static double HOPPER_OPEN_POS = 0.32;
    public static double HOPPER_PUSH_POS = 0.22;

    public static final double deadzone = 0.49;

    public static final double LAUNCHER_THEORETICAL_MAX = 3387.096774;

    public static int WOBBLE_STORE = 0;
    public static int WOBBLE_UP = 977;
    public static int WOBBLE_DOWN = 3110;

    public static double highGoalSpeed = 0.54;
    public static double powerShotSpeed = 0.50;
    public static double wobblePower = 1.0;
    public static double intakePower = 1.0;

    /*
        PowerSpeed is meant to be the speed given to the motor, from [-1,1]
        which the controller PID interprets as a objective rate in ticks per second.
     */
    public static int getLaunchTicksPerSecondFromPowerSpeed(double powerSpeed) {
        return (int) (LAUNCHER_THEORETICAL_MAX * powerSpeed);
    }
}