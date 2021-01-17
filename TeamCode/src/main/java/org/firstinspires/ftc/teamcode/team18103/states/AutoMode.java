package org.firstinspires.ftc.teamcode.team18103.states;

public enum AutoMode {
    NoRings(60),
    OneRing(84),
    FourRings(108);

    private final double dist;

    AutoMode(double dist) {
        this.dist = dist;
    }

    public double getDist() {
        return dist;
    }

}
