package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.Utility.VectorMath.average;

public class ElapsedTimer extends ElapsedTime {

    private ElapsedTime period = new ElapsedTime();
    private Vector<Double> pastPeriods = new Vector<>();

    public double updatePeriodTime(){
        pastPeriods.add(period.seconds());
        period.reset();
        if (pastPeriods.size()>= 200) {
            pastPeriods.remove(0);
        }
        return average(pastPeriods);
    }

    public double getAveragePeriodSec() {
        return average(pastPeriods);
    }

    public double getMaxPeriodSec() {
        return Collections.max(pastPeriods);
    }

    public double getLastPeriodSec() {
        if (pastPeriods.size() != 0) {
            return pastPeriods.lastElement();
        } else {
            return 0;
        }
    }
}
