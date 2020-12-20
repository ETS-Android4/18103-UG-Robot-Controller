package org.firstinspires.ftc.teamcode.lib.geometry;

public class Point {

    private double x;
    private double y;
    private double z;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getXYDist(Point target) {
        return Math.hypot(x - target.x, y - target.y);
    }

    public double getDist(Point target) {
        return Math.hypot(getXYDist(target), z - target.z);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}
