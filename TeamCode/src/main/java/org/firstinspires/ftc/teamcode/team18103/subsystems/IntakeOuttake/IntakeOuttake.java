package org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.geometry.Point;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public class IntakeOuttake extends Subsystem {

    DcMotorEx intake;
    DcMotorEx transfer;
    DcMotorEx firstOuttake;
    DcMotorEx secondOuttake;

    @Override
    public void init(HardwareMap ahMap) {
        intake = ahMap.get(DcMotorEx.class, Constants.intake);
        transfer = ahMap.get(DcMotorEx.class, Constants.transfer);
        firstOuttake = ahMap.get(DcMotorEx.class, Constants.firstOuttake);
        secondOuttake = ahMap.get(DcMotorEx.class, Constants.secondOuttake);

        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        firstOuttake.setDirection(DcMotorEx.Direction.REVERSE);
        secondOuttake.setDirection(DcMotorEx.Direction.REVERSE);
        firstOuttake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        secondOuttake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    // Intake Methods

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

    // Transfer Methods

    public void runTransfer(double power) {
        transfer.setPower(power);
    }

    public double getTransferPower() {
        return transfer.getPower();
    }

    public void runTransfer(boolean on) {
        if (on) {
            runTransfer(1);
        } else {
            stopTransfer();
        }
    }

    public void stopTransfer() {
        runTransfer(0);
    }

    // Outtake Methods

    public void runOuttake(double power) {
        firstOuttake.setPower(power);
        secondOuttake.setPower(power);
    }

    public void runOuttake(boolean on) {
        if (on) {
            runOuttake(0.5);
        } else {
            stopOuttake();
        }
        //updateDiagnostics();
    }

    public void PIDOuttake(double omega) {
        PIDSVA controller = new PIDSVA(0.5, 0, 0, 0d, 0, 0);
        double error = omega;
        while (Math.abs(error) > 10) {
            error = omega - secondOuttake.getVelocity();
            System.out.println(secondOuttake.getVelocity());
            double output = controller.getOutput(error, 0, 0);
            getFirstOuttake().setPower(output);
            getSecondOuttake().setPower(output);
        }

    }

    public void OuttakeFromPoint() {
        //Point targetGoal = Constants.leftGoal;

        //if (location.getX() > 0) {
        //targetGoal = Constants.rightGoal;
        //}

        double dw = 70.75 * Constants.mmPerInch / 1000; //location.getXYDist(targetGoal);
        double dz = (35.5 - 14.5) * Constants.mmPerInch / 1000; //targetGoal.getZ() - location.getZ();

        double v = Math.sqrt((9.8 * (dw * dw)) / (2 * Math.cos(Math.toRadians(Constants.theta)) *
                (dw * Math.sin(Math.toRadians(Constants.theta)) -
                        dz * Math.cos(Math.toRadians(Constants.theta)))));

        double omega =  (v * 60 * 2 * 2) / (Constants.wheelDiam * Math.PI);

        PIDOuttake(omega);

    }

    public double[] shooterDiagnostics() {
        return new double[]{getFirstOuttake().getVelocity(), getSecondOuttake().getVelocity()};
    }

    public void stopOuttake() {
        runOuttake(0);
    }

    public DcMotorEx getFirstOuttake() {
        return firstOuttake;
    }

    public DcMotorEx getSecondOuttake() {
        return secondOuttake;
    }

    public DcMotorEx getIntake() {
        return intake;
    }

    public DcMotorEx getTransfer() {
        return transfer;
    }

}
