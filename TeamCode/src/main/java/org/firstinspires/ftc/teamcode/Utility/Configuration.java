package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Configuration {

    public static double HOPPER_OPEN_POS = 0.32;
    public static double HOPPER_PUSH_POS = 0.22;
    public static double LAUNCH_SERVO_DELAY = 0.3;

    public static final double deadzone = 0.49;

    public static final double LAUNCHER_THEORETICAL_MAX = 3387.096774;

    public static int WOBBLE_STORE = 0;
    public static int WOBBLE_UP = 977;
    public static int WOBBLE_DOWN = 3110;

    public static double highGoalSpeed = 0.54;
    public static double powerShotSpeed = 0.50;
    public static double wobblePower = 1.0;
    public static double intakePower = 1.0;

    public static double launchMeanErrorTicksPerSecond = 80.0;
    public static double launchStdDevErrorTicksPerSecond = 7.0;
    public static int launchErrorWindowSteps = 10;

    // Launcher control parameters
    public static PIDFCoefficients pidfCoeffFeedForward =  new PIDFCoefficients(40,0,0,15);
    public static PIDFCoefficients pidfCoeffOriginalIntegral =  new PIDFCoefficients(10,3,0,0);
    public static double minVelocityToCalibrateTPS = 100; // Min speed to calibrate launcher feed forward
    public static double minVelocityErrorToCalibrateTPS = 30;
    public static double F_increment_scale = 1.0; // Scale calculated increment by this before adding it
    public static double F_overshoot_scale = 0.5; // Scale down increment if error grows
    public static double F_min = 5.0;
    public static double F_max = 30.0;

    /*
        PowerSpeed is meant to be the speed given to the motor, from [-1,1]
        which the controller PID interprets as a objective rate in ticks per second.
     */
    public static int getLaunchTicksPerSecondFromPowerSpeed(double powerSpeed) {
        return (int) (LAUNCHER_THEORETICAL_MAX * powerSpeed);
    }
}