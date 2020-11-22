package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;

public class Transfer extends Subsystem {
    DcMotorEx transfer;

    public Transfer() {

    }

    @Override
    public void init(HardwareMap ahMap) {
        transfer = ahMap.get(DcMotorEx.class, Constants.transfer);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void runTransfer(double power) {
        transfer.setPower(power);
    }

    public void runTransfer(boolean on) {
        if (on) {
            runTransfer(1);
        } else {
            stopIntake();
        }
    }

    public void stopIntake() {
        runTransfer(0);
    }

}
