package org.firstinspires.ftc.teamcode.team18103.states;

public enum AutoMode {
    None(36),
    One(60),
    Four(84);

    private final double dist;

    AutoMode(double dist) {
        this.dist = dist;
    }

    public double getDist() {
        return dist;
    }

}
