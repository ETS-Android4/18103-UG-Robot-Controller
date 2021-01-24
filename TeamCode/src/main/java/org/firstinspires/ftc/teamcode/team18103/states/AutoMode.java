package org.firstinspires.ftc.teamcode.team18103.states;

public enum AutoMode {
    None(40, -45),
    One(64, 45),
    Four(88, -45);

    private final double dist;
    private final double angle;

    AutoMode(double dist, double angle) {
        this.dist = dist;
        this.angle = angle;
    }

    public double getDist() {
        return dist;
    }

    public double getAngle() {return angle;}

}
