package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@TeleOp
public class NthDegreeOpMode extends OpMode {

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
        //robot.loop(telemetry);
        
        telemetry.addData("Left: ", robot.getDriveSubsystem().frontLeft.getCurrentPosition());
        telemetry.addData("Right: ", robot.getDriveSubsystem().frontRight.getCurrentPosition());
        telemetry.addData("Horizontal: ", robot.getDriveSubsystem().backRight.getCurrentPosition());
        telemetry.addData("Chosen One: ", robot.getDriveSubsystem().backLeft.getCurrentPosition());

        robot.getDriveSubsystem().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Sport);

        robot.getIntakeSubsystem().runIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.right_bumper) {
            robot.getTransferSubsystem().runTransfer(robot.getTransferSubsystem().getTransferPower() == -1 ? 0 : -1);
        } else if(gamepad1.left_bumper) {
            robot.getTransferSubsystem().runTransfer(robot.getTransferSubsystem().getTransferPower() == 1 ? 0 : 1);
        }

        robot.getOuttakeSubsystem().runOuttake(gamepad1.y);

        //robot.getWobbleSubsystem().moveJoint(gamepad1.right_trigger - gamepad1.left_trigger);
        //robot.getWobbleSubsystem().moveLatch(gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? 0 : -1);
    }

    public void launchShooter(boolean shooting) {
        robot.getOuttakeSubsystem().runOuttake(shooting);
        robot.getTransferSubsystem().runTransfer(shooting);
    }
}
