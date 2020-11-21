package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;

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
        robot.loop(telemetry);

        robot.getDriveSubsystem().ultimateDriveController(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.right_trigger,
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x);

        robot.getIntakeSubsystem().runIntake(gamepad1.a);
        robot.getTransferSubsystem().runTransfer(gamepad1.dpad_down ? 1 : gamepad1.dpad_up ? -1 : 0);
        robot.getOuttakeSubsystem().runOuttake(gamepad1.y);

    }

    public void launchShooter(boolean shooting) {
        robot.getOuttakeSubsystem().runOuttake(shooting);
        robot.getTransferSubsystem().runTransfer(shooting);
    }
}
