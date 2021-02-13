package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@TeleOp
public class Aimbot extends OpMode {

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

        robot.getDriveSubsystem().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, DriveMode.Sport);

        robot.getIOSubsystem().runIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.right_bumper) {
            robot.getIOSubsystem().runTransfer(-1);
        } else if(gamepad1.left_bumper) {
            robot.getIOSubsystem().runTransfer(1);
        }

        if(gamepad1.y) {

            robot.getDriveSubsystem().rotateToShootingAngle();

            while (gamepad1.y) {
                robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(108 - robot.getDriveSubsystem().getDataFusionX(),
                        24)); // 45 - x, y - 36 for mid goal

                if(gamepad1.right_bumper) {
                    robot.getIOSubsystem().runTransfer(-1);
                } else if(gamepad1.left_bumper) {
                    robot.getIOSubsystem().runTransfer(1);
                }

                robot.getIOSubsystem().runIntake(gamepad1.right_trigger - gamepad1.left_trigger);

                robot.getDriveSubsystem().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, DriveMode.Sport);

            }

        } else {
            robot.getIOSubsystem().runOuttake(0);
        }

        if (gamepad1.a) {
            robot.getWobbleSubsystem().moveLatch(1.0); //open
        }

        if (gamepad1.b) {
            robot.getWobbleSubsystem().moveJoint(false);
        } else if (gamepad1.x) {
            robot.getWobbleSubsystem().moveJoint(true);
        }


        if(gamepad1.dpad_up) {
            robot.getIOSubsystem().runTransfer(0);
        } else if(gamepad1.dpad_down) {
            robot.getWobbleSubsystem().moveLatch(0.40);
        }

        robot.getDriveSubsystem().zeroCoords(gamepad1.dpad_left);
    }

}
