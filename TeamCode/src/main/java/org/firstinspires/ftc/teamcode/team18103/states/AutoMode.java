package org.firstinspires.ftc.teamcode.team18103.states;

public enum AutoMode {
    NoRings(51),
    OneRing(73),
    FourRings(99);

    private final double dist;

    AutoMode(double dist) {
        this.dist = dist;
    }

    public double getDist() {
        return dist;
    }

}
