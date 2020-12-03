package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

/**
 *  Timing Monitor is a debug tool for discovering the source of slowdowns in code.
 *  This is done by keeping track of the LONGEST time interval observed between labeled checkpoints.
 */

public class TimingMonitor {

    private final RobotHardware opmode;
    private final ElapsedTime timer = new ElapsedTime();
    private HashMap<String, Double> checkpoints = new HashMap<>();
    private boolean enabled = true;

    public TimingMonitor(RobotHardware opmode) {
        this.opmode = opmode;
        this.reset();
    }

    public void reset() {
        enabled = true;
        checkpoints.clear();
        timer.reset();
    }

    public void disable() {
        enabled = false;
    }

    public void enable() {
        enabled = true;
        this.reset();
    }

    public void checkpoint(String checkpointName){
        if(!enabled) return;

        double time;
        double prevTime;

        if(!checkpoints.containsKey(checkpointName)) {
            checkpoints.put(checkpointName, 0.0);
        }

        time = timer.seconds();
        timer.reset();
        prevTime = checkpoints.get(checkpointName);

        if (time > prevTime) {
            checkpoints.put(checkpointName, time);
        }
    }

    public void displayMaxTimes() {
        if(!enabled) {
            opmode.telemetry.addLine("TimingMonitor Disabled");
            return;
        }
        opmode.telemetry.addLine("----Timing Monitor Results----");
        checkpoints.forEach((k, v) -> opmode.telemetry.addData(k + ": ", opmode.df_precise.format(v)));
    }
}