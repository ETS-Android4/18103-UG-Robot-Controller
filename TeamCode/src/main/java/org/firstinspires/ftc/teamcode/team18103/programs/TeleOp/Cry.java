package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@TeleOp
public class Cry extends OpMode {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor[] driveMotors;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, Constants.transfer);
        frontRight = hardwareMap.get(DcMotorEx.class, Constants.frontRight);
        backLeft = hardwareMap.get(DcMotorEx.class, Constants.backLeft);
        backRight = hardwareMap.get(DcMotorEx.class, Constants.backRight);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        for (DcMotor motor : driveMotors) {
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        telemetry.addData("Left: ", frontLeft.getCurrentPosition());
        telemetry.addData("Right: ", frontRight.getCurrentPosition());
        telemetry.addData("Horizontal: ", backRight.getCurrentPosition());
        telemetry.addData("Chosen One: ", backLeft.getCurrentPosition());

    }

}
