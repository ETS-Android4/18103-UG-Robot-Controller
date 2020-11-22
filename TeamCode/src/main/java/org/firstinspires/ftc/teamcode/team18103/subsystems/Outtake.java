package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;

public class Outtake extends Subsystem {
    DcMotorEx outtake;
    CRServo transOut;

    public Outtake() {

    }

    @Override
    public void init(HardwareMap ahMap) {
        outtake = ahMap.get(DcMotorEx.class, Constants.outtake);
        transOut = ahMap.get(CRServo.class, Constants.transOut);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void runOuttake(double power) {
        outtake.setPower(power);
        transOut.setPower(power);
    }

    public void runOuttake(boolean on) {
        if (on) {
            runOuttake(1);
        } else {
            stopIntake();
        }
    }

    public void stopIntake() {
        runOuttake(0);
    }

}