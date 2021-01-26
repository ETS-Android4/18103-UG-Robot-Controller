package org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Point;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

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

        secondOuttake.setVelocityPIDFCoefficients(200,0, 0, 12.5);
        firstOuttake.setVelocityPIDFCoefficients(200,0, 0, 12.5);

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
            runTransfer(-1);
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
        PIDSVA controller = new PIDSVA(0.0005, 0, 0, 0d, 0, 0);
        double error = omega;
        double output = 0;
        while (Math.abs(error) >= 20) {
            error = omega - secondOuttake.getVelocity();
            output = controller.getOutput(error, 0, 0);
            System.out.println(getSecondOuttake().getVelocity());
            getFirstOuttake().setPower(getSecondOuttake().getPower() + output);
            getSecondOuttake().setPower(getSecondOuttake().getPower() + output);
        }
        //runOuttake(false);
    }

    public void PIDOuttake2(double omega) {
        secondOuttake.setVelocity(omega);
        firstOuttake.setVelocity(omega);
    }

    public double OuttakeFromPoint(Point location) {
        Point targetGoal = Constants.leftGoal;
        if (location.getX() > 0) {
         targetGoal = Constants.rightGoal;
        }
        double dw = location.getXYDist(targetGoal); //108 * Constants.mmPerInch / 1000;
        double dz = targetGoal.getZ() - location.getZ(); //(35.5 - 14.5) * Constants.mmPerInch / 1000;
        double v = Math.sqrt((9.8 * (dw * dw)) / (2 * cos(Math.toRadians(Constants.theta)) *
                (dw * sin(Math.toRadians(Constants.theta)) -
                        dz * cos(Math.toRadians(Constants.theta)))));
        double omega =  (v * 60 * 1.5 * 2) / (Constants.wheelDiam * Math.PI);
        omega *= 28.0/60;
        if (omega > 1953.5) {
            omega = 1953.5 + (omega - 1953.5) * 1/500;
        }
        //omega *= 2040.0/2939.0;
        PIDOuttake2(omega);
        return omega;
    }

    public double outtakeFromPoint3(double dw) {
        double omega = 120.0/48 * dw + 1650;
        PIDOuttake2(omega);
        return omega;
    }

    public double outtakeFromPoint2() {
        double dw = 70.75 * Constants.mmPerInch / 1000;
        double dz = (35.5 - 14.5) * Constants.mmPerInch / 1000;

        double t = 3.0;
        double v = (dw * Motor.GoBILDA_6000.getTicksPerMM()*1000)/t;
        while (v/cos(toRadians(40)) > 2000 && (dz/t) - 0.5*9.8*t != v/sin(toRadians(40))) {
            v = (dw * Motor.GoBILDA_6000.getTicksPerMM() * 1000)/t;
            t-=0.1;
        }
        t+=0.1;
        v = (dw * Motor.GoBILDA_6000.getTicksPerMM() * 1000)/t;
        return v;
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
