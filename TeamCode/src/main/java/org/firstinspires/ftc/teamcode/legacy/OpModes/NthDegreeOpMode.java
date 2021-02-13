package org.firstinspires.ftc.teamcode.legacy.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@Disabled
@TeleOp
public class NthDegreeOpMode extends OpMode {

    Robot robot = new Robot();
    DcMotor shooter1;
    DcMotor shooter2;
    public boolean wgClose = true;

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry);
        //shooter1 = robot.getOuttakeSubsystem().getFirstOuttake();
        //shooter2 = robot.getOuttakeSubsystem().getSecondOuttake();
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
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    120 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 24)));
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

    public void launchShooter(boolean shooting) {
        //robot.getOuttakeSubsystem().runOuttake(shooting);
        //robot.getTransferSubsystem().runTransfer(shooting);
    }
}
