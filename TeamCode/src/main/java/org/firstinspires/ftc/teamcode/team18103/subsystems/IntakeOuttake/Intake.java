package org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public class Intake extends Subsystem {
    DcMotorEx intake;

    public Intake() {

    }

    @Override
    public void init(HardwareMap ahMap) {
        intake = ahMap.get(DcMotorEx.class, Constants.intake);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void runIntake(boolean on) {
        if (on) {
            runIntake(1);
        } else {
            stopIntake();
        }
    }

    public void stopIntake() {
        runIntake(0);
    }

    public void manuallyPowerIntake(double power) {
        runIntake(power);
    }

    public DcMotorEx getIntake() {
        return intake;
    }

}
