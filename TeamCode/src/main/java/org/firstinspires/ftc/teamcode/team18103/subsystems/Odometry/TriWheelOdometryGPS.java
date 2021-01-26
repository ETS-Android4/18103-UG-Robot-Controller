package org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;

/*
 * Author: Akhil G
 */

public class TriWheelOdometryGPS extends Odometry {

    private DcMotorEx left, right, horizontal;

    private double x = 0, y = 0, theta = 0;
    private double r_0 = 0, l_0 = 0, s_0 = 0, lPos, rPos, sPos;
    private final double ticksPerInch = Motor.REV_Encoder.getTicksPerInch(35);

    public TriWheelOdometryGPS() {}

    public TriWheelOdometryGPS(double x0, double y0, double theta0) {
        x = x0;
        y = y0;
        theta = theta0;
    }

    @Override
    public void init(HardwareMap ahMap) {
        left = ahMap.get(DcMotorEx.class, Constants.left);
        right = ahMap.get(DcMotorEx.class, Constants.right);
        horizontal = ahMap.get(DcMotorEx.class, Constants.horizontal);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void run() {
        // Get Current Positions
        lPos = (getLeft().getCurrentPosition() / getTicksPerInch());
        rPos = (getRight().getCurrentPosition() / getTicksPerInch());

        double dl = lPos - getL_0();
        double dr = rPos - getR_0();

        // Calculate Angle
        double dTheta = (dl - dr) / (Constants.ENCODER_DIFFERENCE);
        theta += dTheta;

        //Get the components of the motion
        double sPose = (getHorizontal().getCurrentPosition() / getTicksPerInch());
        double ds = (sPose - getS_0()) - (dTheta * Constants.HORIZONTAL_OFFSET);

        double p = ((dr + dl) / (2));

        //Calculate and update the position values
        //double dx = p * Math.cos(dTheta/2) + ds * Math.sin(dTheta /2);
        double dx = (p/dTheta) * Math.sin(dTheta);
        double dy = -dx * Math.tan(dTheta/2) + ds * Math.cos(dTheta/2);
        dx += ds * Math.sin(dTheta/2);
        //double dy = p * Math.sin(dTheta/2) + ds * Math.cos(dTheta/2);

        x += dx;
        y += dy;

        theta = MathFx.angleWrap(-180, 180, theta);

        setZeros(lPos, rPos, sPos);
    }

    @Override
    public void update() {
        run();
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getTheta() {
        return theta;
    }

    public DcMotorEx getHorizontal() {
        return horizontal;
    }

    public DcMotorEx getLeft() {
        return left;
    }

    public DcMotorEx getRight() {
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

    public double getlPos() {
        return lPos;
    }

    public double getrPos() {
        return rPos;
    }

    public double getsPos() {
        return sPos;
    }

}
