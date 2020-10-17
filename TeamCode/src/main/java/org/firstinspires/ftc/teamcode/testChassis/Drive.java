package org.firstinspires.ftc.teamcode.testChassis;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfileGenerator;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;

public class Drive {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx[] driveMotors;
    IMU imu;

    public Drive(IMU imu) {
        this.imu = imu;
    }

    public void init(HardwareMap hMap) {
        frontLeft = hMap.get(DcMotorEx.class, Constants.frontLeft);
        frontRight = hMap.get(DcMotorEx.class, Constants.frontRight);
        backLeft = hMap.get(DcMotorEx.class, Constants.backLeft);
        backRight = hMap.get(DcMotorEx.class, Constants.backRight);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        for (DcMotorEx motor : driveMotors) {
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

    }

    public void setDriveMotors(double power) {
        for (DcMotorEx i : driveMotors) {
            i.setPower(power);
        }
    }

    public void setRotateMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    public void distanceddrive(int distance) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < distance) {
            setDriveMotors(.75);
        }
    }

    public void PointRotation(double heading) {
        double curHead = imu.getHeading();

        double error = heading - curHead;

        double distance = Constants.ENCODER_DIFFERENCE * Math.PI * error / 360;

        TrapezoidalMotionProfileGenerator motionProfile = new TrapezoidalMotionProfileGenerator(distance, Motor.GoBILDA_435);
        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        ElapsedTime timer = new ElapsedTime();
        /*
        PIDSVA controller = new PIDSVA(10d/12d, 1d/12d, 10d/12d, 0d, 1d/motionProfile.getMaxV(), 0.01d/12d);
        while (timer.seconds() < motionProfile.getTotalTime()) {
            double timeStamp = timer.seconds();
            double position = motionProfile.getPosition(timeStamp);
            double velocity = motionProfile.getVelocity(timeStamp);
            double acceleration = motionProfile.getAcceleration(timeStamp);
            double pos_error = position - (frontLeft.getCurrentPosition()/ Motor.GoBILDA_435.getTicksPerInch());
            double output = controller.getOutput(pos_error, velocity, acceleration);

            setRotateMotors(output);

            //setDriveState(motionProfile.getProfileState(timeStamp));

        }

         */
        timer.startTime();
        while (timer.seconds() < motionProfile.getTotalTime()) {
            setRotateMotors(motionProfile.getVelocity(timer.milliseconds()/1000));
        }
        setDriveMotors(0);
    }

}
