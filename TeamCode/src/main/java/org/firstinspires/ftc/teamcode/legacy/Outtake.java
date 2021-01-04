package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Point;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public class Outtake extends Subsystem {

    /**
     * Akhil, I don't really know what to tell you about this code.
     * The intitial shoot motors didn't run at a set speed but instead were ACCELERATING TO FULL POWER
     * Hence the video of the motor running at "0.005" power.
     * However, with everything commented, it worked as you would normally expect. (you can also see my test in cry)
     * I have a feeling it has something to do with the run diagnostics in the update and runOuttake method
     *
     * The PID stuff, although commented wasn't being used so I don't think that was the main issue.
     *
     * To David: Alr, I saw this last rip, we can deal w/ it later ig
     */


    DcMotor firstOuttake;
    DcMotor secondOuttake;

    @Override
    public void init(HardwareMap ahMap) {
        firstOuttake = ahMap.get(DcMotor.class, Constants.firstOuttake);
        secondOuttake = ahMap.get(DcMotor.class, Constants.secondOuttake); //Needed a simple name and was lazy :/
        firstOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        secondOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        firstOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        //updateDiagnostics();
    }

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

//    public void PIDOuttake(double omega) {
//        PIDSVA controller = new PIDSVA(0.5, 0, 0, 0d, 0, 0);
//        double error = omega;
//        while (error > 10) {
//            error = omega - getFirstOuttakeRPM();
//            double output = controller.getOutput(error, 0, 0);
//        }
//    }

//    public void OuttakeFromPoint(Point location) {
//        Point targetGoal = Constants.leftGoal;
//
//        if (location.getX() > 0) {
//            targetGoal = Constants.rightGoal;
//        }
//
//        double dw = location.getXYDist(targetGoal);
//        double dz = targetGoal.getZ() - location.getZ();
//
//        double v = Math.sqrt((9.8*(dw*dw))/(2 * Math.cos(Constants.theta) *
//                (dw*Math.sin(Constants.theta)-dz*Math.cos(Constants.theta))));
//
//        double omega = (v*60*2*2)/(Constants.wheelDiam*Math.PI);
//
//        PIDOuttake(omega);
//
//    }

//    private void updateDiagnostics() {
//        firstOuttakeRPM = ((firstOuttake.getCurrentPosition() - firstOuttakelastPos)/ Motor.GoBILDA_6000.getENCODER_TICKS_PER_REVOLUTION())/(elapsedTime.seconds()/60);
//        secondOuttakeRPM = ((secondOuttake.getCurrentPosition() - secondOuttakeLastPos)/Motor.GoBILDA_6000.getENCODER_TICKS_PER_REVOLUTION())/(elapsedTime.seconds()/60);
//        elapsedTime.reset();
//        firstOuttakelastPos = firstOuttake.getCurrentPosition();
//        secondOuttakeLastPos = secondOuttake.getCurrentPosition();
//    }

    public void stopOuttake() {
        runOuttake(0);
    }

//    public double getFirstOuttakeRPM() {
//        return firstOuttakeRPM;
//    }
//
//    public double getSecondOuttakeRPM() {
//        return secondOuttakeRPM;
//    }

    public DcMotor getFirstOuttake() {
        return firstOuttake;
    }

    public DcMotor getSecondOuttake() {
        return secondOuttake;
    }

//    public double getFirstOuttakelastPos() {
//        return firstOuttakelastPos;
//    }
//
//    public double getSecondOuttakeLastPos() {
//        return secondOuttakeLastPos;
//    }
}
