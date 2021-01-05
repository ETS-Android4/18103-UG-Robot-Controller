package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

/*
 * Author: Akhil G
 *
 * Controls
 *
 * Chassis:
 *
 * Forward/Backward (GamePad Left Stick y)
 * Left/Right (Strafe) (GamePad Left Stick x)
 * (Turn) Rotational Force (GamePad Right Stick x)
 * Lower Gear Adjustment (GamePad Left Trigger)
 * Higher Gear Adjustment (GamePad Right Trigger)
 * Field-Centric Drive Setter (GamePad Left Bumper) * Don't Use
 * Point-of-View Drive Setter (GamePad Right Bumper) * Don't Use
 * Zero Yaw Setter (GamePad X Button)
 *
 * Intake-Outtake:
 *
 * Run at full speed (GamePad Y Button)
 *
 */

@TeleOp
@Disabled
public class TestChassis extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop(telemetry);

        //robot.getDriveSubsystem().ultimateDriveController(gamepad1.left_stick_y, gamepad1.left_stick_x,
        //        gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.right_trigger,
        //        gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x);

        robot.getDriveSubsystem().setDriveMotors(gamepad1.left_stick_y);
        //robot.getIntakeSubsystem().runIntake(gamepad1.a);
        //robot.getTransferSubsystem().runTransfer(gamepad1.b);
        //robot.getOuttakeSubsystem().runOuttake(gamepad1.y);

    }

}
