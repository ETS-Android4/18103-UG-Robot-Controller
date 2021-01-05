package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@TeleOp
@Disabled
public class Cry extends OpMode {
    DcMotor firstOuttake;
    DcMotor secondOuttake;

    @Override
    public void init() {
        firstOuttake = hardwareMap.get(DcMotor.class, Constants.firstOuttake);
        secondOuttake = hardwareMap.get(DcMotor.class, Constants.secondOuttake);
        firstOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        secondOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        firstOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(gamepad1.y) {
            firstOuttake.setPower(0.5);
            secondOuttake.setPower(0.5);
        }
    }


}
