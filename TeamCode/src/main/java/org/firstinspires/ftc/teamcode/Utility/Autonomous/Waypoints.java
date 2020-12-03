package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount;

public class Waypoints {

    private final AllianceColor allianceColor;
    private RingDetectionAmount ringDetectionAmount;

    public Waypoints(AllianceColor allianceColor, RingDetectionAmount ringDetectionAmount) {
        this.allianceColor = allianceColor;
        this.ringDetectionAmount = ringDetectionAmount;

        configureWaypoints(allianceColor, ringDetectionAmount);
    }

    public Waypoints(AllianceColor allianceColor) {
        this(allianceColor, RingDetectionAmount.ZERO);
    }

    private void configureWaypoints(AllianceColor allianceColor, RingDetectionAmount ringDetectionAmount) {
        if(allianceColor.equals(AllianceColor.BLUE)) {
            for(Positions position : Positions.values())
                position.reflectInX();
        }

        for(Positions position : Positions.values())
            position.setLabel(position.name());
    }

    public void setRingDetectionAmount(RingDetectionAmount ringDetectionAmount) {
        this.ringDetectionAmount = ringDetectionAmount;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public RingDetectionAmount getRingDetectionAmount() {
        return ringDetectionAmount;
    }
}