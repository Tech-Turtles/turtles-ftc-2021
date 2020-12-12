package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;

/**
 * @author Christian
 *
 * Enum with hardcoded waypoint positions.
 * Positions can be modified later in code.
 *
 * All positions are based off the red side.
 */
public enum Positions {
    START_WALL(-62,-42,180),
    START_CENTER(-62,-24,0),
    RINGS(-24,-36,0),
    ZONE_A(12,-60,0),
    ZONE_B(-36,36,0),
    ZONE_C(-60,60,0),
    PARK(12,-42,180);

    private double x;
    private double y;
    private double theta;
    private MecanumNavigation.Navigation2D nav2D;

    /**
     * @param x Waypoint X coordinate
     * @param y Waypoint Y coordinate
     * @param theta Waypoint Theta, Degrees CCW
     */
    Positions(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = Math.toRadians(theta);
        this.nav2D = getNewNavigation2D();
    }

    public void setX(double x) {
        this.x = x;
        this.nav2D = getNewNavigation2D();
    }

    public void setY(double y) {
        this.y = y;
        this.nav2D = getNewNavigation2D();
    }

    public void setTheta(double theta) {
        this.theta = theta;
        this.nav2D = getNewNavigation2D();
    }

    public MecanumNavigation.Navigation2D getNav2D() {
        return nav2D;
    }

    public MecanumNavigation.Navigation2D getNewNavigation2D(){
        return new MecanumNavigation.Navigation2D(x, y, theta);
    }

    public void reflectInX() {
        nav2D.reflectInX();
    }

    public void setLabel(String label) {
        nav2D.setLabel(label);
    }

    public void set(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.nav2D = getNewNavigation2D();
    }

    public void set(MecanumNavigation.Navigation2D navigation2D) {
        this.x = navigation2D.x;
        this.y = navigation2D.y;
        this.theta = navigation2D.theta;
        this.nav2D = getNewNavigation2D();
    }
}