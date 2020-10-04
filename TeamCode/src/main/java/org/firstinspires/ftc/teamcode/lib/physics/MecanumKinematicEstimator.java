package org.firstinspires.ftc.teamcode.lib.physics;

/*
 * Author: Akhil G
 */

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public class MecanumKinematicEstimator extends Subsystem {

    private double x, y, theta;
    private ElapsedTime time;
    private double t_0;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private double fl, fr, bl, br, fl_0, fr_0, bl_0, br_0;

    public MecanumKinematicEstimator() {
        x = 0;
        y = 0;
        theta = 0;
        t_0 = 0;
    }

    public MecanumKinematicEstimator(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        t_0 = 0;
    }

    @Override
    public void init(HardwareMap ahMap) {
        frontLeft = ahMap.get(DcMotorEx.class, Constants.frontLeft);
        frontRight = ahMap.get(DcMotorEx.class, Constants.frontRight);
        backLeft = ahMap.get(DcMotorEx.class, Constants.backLeft);
        backRight = ahMap.get(DcMotorEx.class, Constants.backRight);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void start() {
        time.startTime();
        fl_0 = 0;
        bl_0 = 0;
        fr_0 = 0;
        br_0 = 0;
    }

    public void run() {
        double dt = (time.milliseconds() - t_0)/1000;
        t_0 = time.milliseconds();

        fl = frontLeft.getVelocity();
        fr = frontRight.getVelocity();
        bl = backLeft.getVelocity();
        br = backRight.getVelocity();

        fl += fl_0;
        fr += fr_0;
        bl += bl_0;
        br += br_0;

        fl /= 2;
        fr /= 2;
        bl /= 2;
        br /= 2;

        double dx = (bl + fr - (fl + br))/4;
        double dy = (bl + fr + fl + br)/4;
        double dTheta = (fl + bl - (fr + br))/(4 * Constants.ENCODER_DIFFERENCE * Math.PI / 360);

        x += dx * dt;
        y += dy * dt;
        theta += dTheta * dt;

        fl_0 = frontLeft.getVelocity();
        fr_0 = frontRight.getVelocity();
        bl_0 = backLeft.getVelocity();
        br_0 = backRight.getVelocity();

    }

    @Override
    public void update() {
        run();
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public double getTheta() {
        return theta;
    }

}
