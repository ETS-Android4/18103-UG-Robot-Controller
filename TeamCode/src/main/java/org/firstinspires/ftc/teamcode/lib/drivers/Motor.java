package org.firstinspires.ftc.teamcode.lib.drivers;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;

/*
 * Author: Akhil G
 */

public enum Motor {
    GoBILDA_1150(1150d, 145.6d),
    GoBILDA_435(435d, 383.6d),
    GoBILDA_312(312d, 537.6d),
    GoBILDA_223(223d, 753.2d),
    GoBILDA_6000(6000d, 28),
    NeveRest_3_7(1780d, 103.6d),
    REV_Core_Hex(125d, 288d),
    REV_Encoder(10000,8129);

    private final double RPM;
    private final double TicksPerRev;

    Motor(double RPM, double encoderTicksPerRevolution) {
        this.RPM = RPM;
        this.TicksPerRev = encoderTicksPerRevolution;
    }

    public double maxAngularVelocity() {
        return getRPM() * Math.PI / 30d;
    }

    public double getRPM() {
        return RPM;
    }

    public double getTicksPerRev() {
        return TicksPerRev;
    }

    public double getTicksPerDegree() {
        return (getTicksPerInch() * (Constants.ENCODER_DIFFERENCE * Math.PI)) / 180;
    }

    public double getTicksPerMM() {
        return getTicksPerRev()/(100 * Math.PI);
    }

    public double getTicksPerMM(double diam) {
        return getTicksPerRev()/(diam * Math.PI);
    }

    public double getTicksPerInch() {
        return getTicksPerMM() * Constants.mmPerInch;
    }

    public double getTicksPerInch(double diam) {
        return getTicksPerMM(diam) * Constants.mmPerInch;
    }

    public double getMMPerSec() {
        return getRPM() * 100 * Math.PI/ 60d;
    }
}
