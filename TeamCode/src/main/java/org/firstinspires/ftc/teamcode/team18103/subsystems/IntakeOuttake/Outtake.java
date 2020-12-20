package org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Point;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public class Outtake extends Subsystem {
    DcMotorEx firstOuttake;
    DcMotorEx secondOuttake;
    ElapsedTime elapsedTime;

    double firstOuttakeRPM;
    double secondOuttakeRPM;

    double firstOuttakelastPos;
    double secondOuttakeLastPos;


    public Outtake() {

    }

    @Override
    public void init(HardwareMap ahMap) {
        firstOuttake = ahMap.get(DcMotorEx.class, Constants.firstOuttake);
        secondOuttake = ahMap.get(DcMotorEx.class, Constants.secondOuttake); //Needed a simple name and was lazy :/
        firstOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        firstOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elapsedTime = new ElapsedTime();
        firstOuttakeRPM = 0;
        secondOuttakeRPM = 0;
        firstOuttakelastPos = 0;
        secondOuttakeLastPos = 0;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        updateDiagnostics();
    }

    public void runOuttake(double power) {
        ElapsedTime timeout = new ElapsedTime();
        firstOuttake.setPower(power);
        secondOuttake.setPower(power);
        //do {} while(firstOuttakeRPM < 4000 || timeout.seconds() > 3);
    }

    public void runOuttake(boolean on) {
        if (on) {
            runOuttake(0.85);
        } else {
            stopIntake();
        }
        updateDiagnostics();
    }

    public void PIDOuttake(double omega) {
        PIDSVA controller = new PIDSVA(0.5, 0, 0, 0d, 0, 0);
        double error = omega;
        while (error > 10) {
            error = omega - getFirstOuttakeRPM();
            double output = controller.getOutput(error, 0, 0);
        }
    }

    public void OuttakeFromPoint(Point location) {
        Point targetGoal = Constants.leftGoal;

        if (location.getX() > 0) {
            targetGoal = Constants.rightGoal;
        }

        double dw = location.getXYDist(targetGoal);
        double dz = targetGoal.getZ() - location.getZ();

        double v = Math.sqrt((9.8*(dw*dw))/(2 * Math.cos(Constants.theta) *
                (dw*Math.sin(Constants.theta)-dz*Math.cos(Constants.theta))));

        double omega = (v*60*2*2)/(Constants.wheelDiam*Math.PI);

        PIDOuttake(omega);

    }

    private void updateDiagnostics() {
        firstOuttakeRPM = ((firstOuttake.getCurrentPosition() - firstOuttakelastPos)/ Motor.GoBILDA_6000.getENCODER_TICKS_PER_REVOLUTION())/(elapsedTime.seconds()/60);
        secondOuttakeRPM = ((secondOuttake.getCurrentPosition() - secondOuttakeLastPos)/Motor.GoBILDA_6000.getENCODER_TICKS_PER_REVOLUTION())/(elapsedTime.seconds()/60);
        elapsedTime.reset();
        firstOuttakelastPos = firstOuttake.getCurrentPosition();
        secondOuttakeLastPos = secondOuttake.getCurrentPosition();
    }

    public void stopIntake() {
        runOuttake(0);
    }

    public double getFirstOuttakeRPM() {
        return firstOuttakeRPM;
    }

    public double getSecondOuttakeRPM() {
        return secondOuttakeRPM;
    }

    public double getFirstOuttakelastPos() {
        return firstOuttakelastPos;
    }

    public double getSecondOuttakeLastPos() {
        return secondOuttakeLastPos;
    }
}
