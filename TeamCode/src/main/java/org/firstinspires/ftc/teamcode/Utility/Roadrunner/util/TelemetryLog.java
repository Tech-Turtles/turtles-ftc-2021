package org.firstinspires.ftc.teamcode.Utility.Roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;

import java.io.Serializable;
import java.util.EnumMap;
import java.util.HashMap;

public class TelemetryLog implements Serializable {
    private double time;
    transient private Pose2d pose;
    private HashMap<String,Double> poseMap = new HashMap<>();
    private EnumMap<Motors,Integer> motorEncoders = new EnumMap<Motors, Integer>(Motors.class);


    public TelemetryLog() {
        this(0.0, new Pose2d(), new EnumMap<Motors, Integer>(Motors.class){{
            put(Motors.FRONT_RIGHT,0);
            put(Motors.FRONT_LEFT,0);
            put(Motors.BACK_RIGHT,0);
            put(Motors.BACK_LEFT,0);
        }});
    }

    public TelemetryLog(double time, Pose2d pose, EnumMap<Motors, Integer> motorEncoders) {
        this.time = time;
        this.pose = pose;
        this.motorEncoders = motorEncoders;
        this.poseMap =
            new HashMap<String, Double>(){{
                put("X",pose.getX());
                put("Y",pose.getY());
                put("Heading",pose.getHeading());
            }};
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void setPoseMap(HashMap<String, Double> poseMap) {
        this.poseMap = poseMap;
    }

    public void setMotorEncoders(EnumMap<Motors, Integer> motorEncoders) {
        this.motorEncoders = motorEncoders;
    }

    public double getTime() {
        return time;
    }

    public Pose2d getPose() {
        return pose;
    }

    public HashMap<String, Double> getPoseMap() {
        return poseMap;
    }

        public EnumMap<Motors, Integer> getMotorEncoders() {
        return motorEncoders;
    }






}
