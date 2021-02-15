package org.firstinspires.ftc.teamcode.Utility.Roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;

import java.io.Serializable;
import java.util.EnumMap;
import java.util.HashMap;

public class TelemetryLog implements Serializable {
    private double time;
    private double x;
    private double y;
    private double heading;
    private int encoder_frontLeft;
    private int encoder_frontRight;
    private int encoder_backLeft;
    private int encoder_backRight;

    public static final String[] fieldOrder = {"TIME","X","Y","HEADING",
        "ENCODER_FRONTLEFT", "ENCODER_FRONTRIGHT", "ENCODER_BACKLEFT", "ENCODER_BACKRIGHT"};

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
        this.x = pose.getX();
        this.y = pose.getY();
        this.heading = Math.toDegrees(pose.getHeading());
        this.encoder_frontRight = motorEncoders.get(Motors.FRONT_RIGHT);
        this.encoder_frontLeft = motorEncoders.get(Motors.FRONT_LEFT);
        this.encoder_backRight = motorEncoders.get(Motors.BACK_RIGHT);
        this.encoder_backLeft = motorEncoders.get(Motors.BACK_LEFT);
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setEncoder_frontLeft(int encoder_frontLeft) {
        this.encoder_frontLeft = encoder_frontLeft;
    }

    public void setEncoder_frontRight(int encoder_frontRight) {
        this.encoder_frontRight = encoder_frontRight;
    }

    public void setEncoder_backLeft(int encoder_backLeft) {
        this.encoder_backLeft = encoder_backLeft;
    }

    public void setEncoder_backRight(int encoder_backRight) {
        this.encoder_backRight = encoder_backRight;
    }


    public double getTime() {
        return time;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public int getEncoder_frontLeft() {
        return encoder_frontLeft;
    }

    public int getEncoder_frontRight() {
        return encoder_frontRight;
    }

    public int getEncoder_backLeft() {
        return encoder_backLeft;
    }

    public int getEncoder_backRight() {
        return encoder_backRight;
    }

}
