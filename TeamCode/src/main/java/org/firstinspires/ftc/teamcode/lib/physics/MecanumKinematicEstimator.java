package org.firstinspires.ftc.teamcode.lib.physics;

/*
 * Author: Akhil G
 */

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public class MecanumKinematicEstimator extends Subsystem {

    double x, y, theta, dx, dy, dTheta;
    ElapsedTime time = new ElapsedTime();
    double t_0;
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx[] driveMotors;
    double fl, fr, bl, br, fl_0, fr_0, bl_0, br_0;

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

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        for (DcMotorEx motor : driveMotors) {
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    @Override
    public void start() {
        fl_0 = 0;
        bl_0 = 0;
        fr_0 = 0;
        br_0 = 0;
    }

    public void run() {
        double fl_1 = frontLeft.getCurrentPosition();
        double fr_1 = frontRight.getCurrentPosition();
        double bl_1 = backLeft.getCurrentPosition();
        double br_1 = backRight.getCurrentPosition();

        fl = fl_1 - fl_0;
        fr = fr_1 - fr_0;
        bl = bl_1 - bl_0;
        br = br_1 - br_0;

        dx = (fl + br - (fr + bl))/4;
        dy = (bl + fr + fl + br)/4;
        dTheta = (fl + bl - (fr + br))/(4);

        dx /= Motor.GoBILDA_435.getTicksPerInch();
        dy /= Motor.GoBILDA_435.getTicksPerInch();
        dTheta /= Motor.GoBILDA_435.getTicksPerDegree();

        x += dx;
        y += dy;
        theta += dTheta;

        theta = MathFx.angleWrap(-180, 180, theta);

        fl_0 = fl_1;
        fr_0 = fr_1;
        bl_0 = bl_1;
        br_0 = br_1;

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

    public DcMotorEx getFrontLeft() {
        return frontLeft;
    }

    public DcMotorEx getFrontRight() {
        return frontRight;
    }

    public DcMotorEx getBackLeft() {
        return backLeft;
    }

    public DcMotorEx getBackRight() {
        return backRight;
    }

    public double getDx() {
        return dx;
    }

    public double getDTheta() {
        return dTheta;
    }

    public double getDy() {
        return dy;
    }

}