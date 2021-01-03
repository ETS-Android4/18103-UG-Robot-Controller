package org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;

/*
 * Author: Akhil G
 */

public class TriWheelOdometryGPS extends Odometry {

    private DcMotor left, right, horizontal;
    private double ticksPerInch, dt;

    private double x = 0, y = 0, theta = 0;
    private double r_0 = 0, l_0 = 0, s_0 = 0;


    //We can only pass by reference or else it breaks.
    public TriWheelOdometryGPS(DcMotor left, DcMotor right, DcMotor horizontal, double ticksPerInch, int dt) {
        this.left = left;
        this.right = right;
        this.horizontal = horizontal;
        this.ticksPerInch = ticksPerInch;
        this.dt = dt;
    }

    public TriWheelOdometryGPS(DcMotor left, DcMotor right, DcMotor horizontal,
                               double ticksPerInch, int dt, double x0, double y0, double theta0) {
        this.left = left;
        this.right = right;
        this.horizontal = horizontal;
        this.ticksPerInch = ticksPerInch;
        this.dt = dt;
        x = x0;
        y = y0;
        theta = theta0;
    }

    public TriWheelOdometryGPS(double ticksPerInch, int dt) {
        this.ticksPerInch = ticksPerInch;
        this.dt = dt;
    }

    @Override
    public void start() {

    }

    @Override
    public void init(HardwareMap ahMap) {
        left = ahMap.get(DcMotor.class, Constants.left);
        right = ahMap.get(DcMotor.class, Constants.right);
        horizontal = ahMap.get(DcMotor.class, Constants.horizontal);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void run() {
        //Get Current Positions
        double lPos = (getLeft().getCurrentPosition() * getTicksPerInch());
        double rPos = (getRight().getCurrentPosition() * getTicksPerInch());

        double dl = lPos - getL_0();
        double dr = rPos - getR_0();

        //Calculate Angle
        double dTheta = (dl - dr) / (Constants.ENCODER_DIFFERENCE);
        theta += dTheta;

        //Get the components of the motion
        double sPose = (getHorizontal().getCurrentPosition() * getTicksPerInch());
        double ds = (sPose - s_0) - (dTheta * Constants.HORIZONTAL_OFFSET);

        double p = ((dr + dl) / (2));

        //Calculate and update the position values
        double dx = p * Math.cos(dTheta/2) + ds * Math.sin(dTheta /2);
        //double dy = -dx * Math.tan(dTheta /2) + ds * Math.cos(dTheta /2);
        double dy = p * Math.sin(dTheta/2) + ds * Math.cos(dTheta/2);

        x += dx;
        y += dy;

        setZeros(lPos, rPos, sPose);
    }

    @Override
    public void update() {
        run();
    }

    @Override
    public double getX() {
        run();
        return x;
    }

    @Override
    public double getY() {
        run();
        return y;
    }

    @Override
    public double getTheta() {
        run();
        return theta;
    }

    public DcMotor getHorizontal() {
        return horizontal;
    }

    public DcMotor getLeft() {
        return left;
    }

    public DcMotor getRight() {
        return right;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getL_0() {
        return l_0;
    }

    public double getR_0() {
        return r_0;
    }

    public double getS_0() {
        return s_0;
    }

    public void setZeros(double l_0, double r_0, double s_0) {
        this.l_0 = l_0;
        this.r_0 = r_0;
        this.s_0 = s_0;
    }

}
