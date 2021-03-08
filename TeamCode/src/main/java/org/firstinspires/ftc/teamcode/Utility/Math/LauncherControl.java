package org.firstinspires.ftc.teamcode.Utility.Math;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Utility.Math.LauncherControl.LaunchControllerState.*;


public class LauncherControl {

    public enum LaunchControllerState {
        Integral, // Original Proportional Integral Control PIDF (10, 3, 0, 0)
        FeedForwardOpenLoop, // Unmonitored Feed Forward Control PIDF(40,0,0,15)
        FeedForwardClosedLoop, // Monitored Feed Forward Control PIDF(40,0,0,X)
    }

    private RobotHardware opmode;
    private LaunchControllerState launchControllerState = FeedForwardClosedLoop;

    public LauncherControl(RobotHardware opmode, LaunchControllerState launchControllerState) {
        this.opmode = opmode;
        setLaunchControllerState(launchControllerState);
    }

    private double F_previous = 0.0;
    private double error_previous = 0.0;
    private double F_increment_previous = 1.0;
    /**
     * If Calibration is needed AND LaunchControllerState = FeedForwardClosedLoop,
     * Calculate an increment for the F parameter and add it, recording previous F and error.
     *
    @return false if calibration is not needed
     */
    public boolean calibrateFeedForward(double targetVelocityTicksPerSecond) {
        boolean calibrationNeeded = isCalibrationNeeded(targetVelocityTicksPerSecond);
        if(calibrationNeeded & launchControllerState == FeedForwardClosedLoop) {
            double F_current = getF();
            double averageVelocityTPS = ListMath.average(opmode.launchVelocityHistory);
            double velocityErrorTPS = targetVelocityTicksPerSecond - averageVelocityTPS;
            double F_new;

            // Might add logic to check if previous calibration error was better
            if(Math.abs(velocityErrorTPS) > Math.abs(error_previous)) {
                // Scale down increment and reapply.
                F_new = F_previous + F_increment_previous * Configuration.F_overshoot_scale;
                F_increment_previous *= Configuration.F_overshoot_scale; // Scale down in case still overshooting
            } else {
          /* How do we calculate the increment to F
          Assumption 1: F_correct / targetVelocity = F_current / currentVelocity
           =>           F_correct = F_current / avgVel * targetVel
           =>           F_increment = (F_current * targetVel / avgVel) - F_current
                Note that this assumes a linear relationship between F and velocity, which is
                probably not true.  However, it may work just the same
           */
                double F_increment = ( F_current * targetVelocityTicksPerSecond / averageVelocityTPS ) - F_current;
                F_increment *= Configuration.F_increment_scale;
                this.F_previous = F_current;
                this.error_previous = velocityErrorTPS;
                this.F_increment_previous = F_increment;
                F_new = F_current + F_increment;
            }
            F_new = Math.min(Math.max(F_new, Configuration.F_min), Configuration.F_max); // Bound F range
            setF(F_new);

        } else {
            error_previous = 100000.0;  // Big, prevents triggering revert on first calibration b/c previous error too low.
        }
        return calibrationNeeded;
    }

    /**
     * Determine if calibration is needed
     @return true if calibration is needed
     */
    public boolean isCalibrationNeeded(double targetVelocityTicksPerSecond) {
        double averageVelocityTPS = ListMath.average(opmode.launchVelocityHistory);
        double velocityErrorTPS = targetVelocityTicksPerSecond - averageVelocityTPS;
        boolean calibrationNeeded = isSteadyState(opmode.launchVelocityHistory)
                & Math.abs(averageVelocityTPS) >  Configuration.minVelocityToCalibrateTPS
                & Math.abs(velocityErrorTPS) >  Configuration.minVelocityErrorToCalibrateTPS;
        return calibrationNeeded;
    }

    public LaunchControllerState getLaunchControllerState() {
        return launchControllerState;
    }

    public void setLaunchControllerState(LaunchControllerState launchControllerState) {
        this.launchControllerState = launchControllerState;
        switch (launchControllerState) {
            case Integral:
                opmode.motorUtility.setPIDFCoefficientsCompensated(Motors.LAUNCHER, DcMotor.RunMode.RUN_USING_ENCODER,
                        Configuration.pidfCoeffOriginalIntegral);
                break;
            case FeedForwardOpenLoop:
            case FeedForwardClosedLoop:
                opmode.motorUtility.setPIDFCoefficientsCompensated(Motors.LAUNCHER, DcMotor.RunMode.RUN_USING_ENCODER,
                        Configuration.pidfCoeffFeedForward);
        }
    }

    private void setF(double f_parameter) {
        PIDFCoefficients coeff = Configuration.pidfCoeffFeedForward;
        opmode.motorUtility.setPIDFCoefficientsCompensated(Motors.LAUNCHER, DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(coeff.p,coeff.i,coeff.d,f_parameter));
    }

    /**
     * @return Return F parameter
     */
    public double getF() {
        return opmode.motorUtility.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Motors.LAUNCHER).f;
    }



    /* **************************STATIC METHODS**********************************
     */

    /*
    Standard Deviation must be below a threshold to be considered steady state.
    This is necessary for launch, and also potentially for tuning the PIDF F parameter
     */
    public static boolean isSteadyState(List<Double> velocityHistory) {
        double standardDeviationOfVelocity = ListMath.standardDeviation(velocityHistory,
                Configuration.launchErrorWindowSteps);
        return standardDeviationOfVelocity < Configuration.launchStdDevErrorTicksPerSecond;
    }

    /**
    Indicates that we are at steady state and the steady state error is within acceptable limits.
     */
    public static boolean isErrorLow(List<Double> velocityHistory, double targetVelocityTicksPerSecond) {
        boolean steadyState = isSteadyState(velocityHistory);
        double velocityAverage = ListMath.average(velocityHistory,
                Configuration.launchErrorWindowSteps);
        double averageError = Math.abs( velocityAverage - targetVelocityTicksPerSecond);
        return steadyState & (averageError < Configuration.launchMeanErrorTicksPerSecond);
    }

   /*
    Error low, and firing servo is ready,
     */
    public static boolean isReadyToShoot(List<Double> velocityHistory, double targetVelocityTicksPerSecond,
                                         double fireTimestamp, double resetTimestamp, double time) {
        boolean steadyState = isSteadyState(velocityHistory);
        boolean errorLow = isErrorLow(velocityHistory, targetVelocityTicksPerSecond);
        boolean fireServoReady = fireTimestamp < resetTimestamp  &
                resetTimestamp + Configuration.LAUNCH_SERVO_DELAY < time;
        return steadyState & errorLow & fireServoReady;
    }


}
