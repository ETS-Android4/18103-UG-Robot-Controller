package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

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
public class TestRobotAutoAim extends OpMode {
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

        robot.getDriveSubsystem().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Balanced); // Max Speed

        robot.getDriveSubsystem().zeroCoords(gamepad1.right_bumper);

        robot.getIOSubsystem().runIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        robot.getIOSubsystem().runTransfer(-(gamepad1.right_trigger-gamepad1.left_trigger));

        //telemetry.addData("Target Theta", robot.getDriveSubsystem().rotateToShootingAngle());

        if(gamepad1.y) {
            robot.getDriveSubsystem().rotateToShootingAngle();

            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    120 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 24)));

        } else {
            robot.getIOSubsystem().runOuttake(false);
        }

        // WBS Commands from NthDegreeOpMode

        if (gamepad1.a) {
            robot.getWobbleSubsystem().moveLatch(1.0); //open
            //if (wgClose = false) {
            //robot.getWobbleSubsystem().moveLatch(0.40);
            //wgClose = true;
            //} else {
            //robot.getWobbleSubsystem().moveLatch(1);
            //wgClose = false;
            //}
        }

        /*if (gamepad1.a && wgClose) {
            robot.getWobbleSubsystem().moveLatch(1.0); //open
            wgClose = false;
        } else if (gamepad1.a && !wgClose) {
            robot.getWobbleSubsystem().moveLatch(0.40);
            wgClose = true;
        }
        Toggle --> if wg is open, a closes; if wg is closed, a opens
        */

        if (gamepad1.b) {
            robot.getWobbleSubsystem().moveJoint(false);
        } else if (gamepad1.x) {
            robot.getWobbleSubsystem().moveJoint(true);
        }

        /*if(gamepad1.dpad_left) {
            robot.getWobbleSubsystem().moveJoint(true);
        } else if(gamepad1.dpad_right) {
            robot.getWobbleSubsystem().moveJoint(false);
        }*/

        if(gamepad1.dpad_up) {
            //robot.getWobbleSubsystem().moveLatch(1);
        } else if(gamepad1.dpad_down) {
            robot.getWobbleSubsystem().moveLatch(0.40);
        }

    }

}
