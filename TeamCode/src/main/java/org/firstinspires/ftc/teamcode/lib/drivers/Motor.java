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
    NeveRest_3_7(1780d, 103.6d),
    REV_Core_Hex(125d, 288d);

    private final double RPM;
    private final double ENCODER_TICKS_PER_REVOLUTION;

    Motor(double RPM, double encoderTicksPerRevolution) {
        this.RPM = RPM;
        this.ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
    }

    public double maxAngularVelocity() {
        return getRPM() * Math.PI / 30d;
    }

    public double getRPM() {
        return RPM;
    }

    public double getENCODER_TICKS_PER_REVOLUTION() {
        return ENCODER_TICKS_PER_REVOLUTION;
    }

    public double getTicksPerMM() {
        return getENCODER_TICKS_PER_REVOLUTION()/(100 * Math.PI);
    }

    public double getTicksPerInch() {
        return getTicksPerMM()/ Constants.mmPerInch;
    }

    public double getMMPerSec() {
        return getRPM() * 100 * Math.PI/ 60d;
    }
}
