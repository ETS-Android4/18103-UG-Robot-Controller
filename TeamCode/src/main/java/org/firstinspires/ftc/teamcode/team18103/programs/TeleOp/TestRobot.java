package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
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
public class TestRobot extends OpMode {
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

        /*telemetry.addData("Velocity 2:", robot.getIOSubsystem().getSecondOuttake().getVelocity());
        telemetry.addData("Velocity 1:", robot.getIOSubsystem().getSecondOuttake().getVelocity());*/

        /*robot.getDriveSubsystem().ultimateDriveController(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.right_trigger,
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x);*/

        robot.getDriveSubsystem().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Balanced); // Half Speed

        robot.getDriveSubsystem().zeroCoords(gamepad1.a);

        robot.getIOSubsystem().runIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.y) {
            telemetry.addData("Omega", robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    120 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36))));
        }

        robot.getIOSubsystem().runTransfer(-(gamepad1.left_trigger));

        /*

        if(gamepad1.right_bumper) {
            robot.getIOSubsystem().runTransfer(robot.getIOSubsystem().getTransferPower() == -1 ? 0 : -1);
        } else if(gamepad1.left_bumper) {
            robot.getIOSubsystem().runTransfer(robot.getIOSubsystem().getTransferPower() == 1 ? 0 : 1);
        }

         */

        /*
        if(gamepad1.y) {
            robot.getIOSubsystem().runOuttake(1);
        } else {
            robot.getIOSubsystem().runOuttake(0);
        }

         */

    }

}
