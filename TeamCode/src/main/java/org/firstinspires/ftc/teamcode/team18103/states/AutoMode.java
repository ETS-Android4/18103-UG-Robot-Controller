package org.firstinspires.ftc.teamcode.team18103.states;

public enum AutoMode {
    NoRings(36),
    OneRing(60),
    FourRings(84);

    private final double dist;

    AutoMode(double dist) {
        this.dist = dist;
    }

    public double getDist() {
        return dist;
    }

}
