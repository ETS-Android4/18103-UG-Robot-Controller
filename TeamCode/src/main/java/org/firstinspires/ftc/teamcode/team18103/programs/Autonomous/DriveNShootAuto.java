package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@Autonomous
public class DriveNShootAuto extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        robot.start();

        while (Math.abs(75 - robot.getDriveSubsystem().getDataFusionY()) > 15) {
            robot.getDriveSubsystem().POVMecanumDrive(-1 + (robot.getDriveSubsystem().getDataFusionY())/75, 0, 0, DriveMode.Balanced);
            robot.loop(telemetry);
        }

        robot.getDriveSubsystem().setDriveMotors(0);

        /*robot.getDriveSubsystem().setDriveMotors(0.5);
        do {
            //run
        } while (robot.getDriveSubsystem().getDataFusionY() < 72);
        robot.getDriveSubsystem().setDriveMotors(0);

         */
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < 1.5) {
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    108 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
        }
        while (timer.seconds() < 5) {
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    108 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            robot.getIOSubsystem().runTransfer(true);
        }

        robot.getIOSubsystem().runTransfer(false);
        robot.getIOSubsystem().runOuttake(false);

        //robot.getIOSubsystem().OuttakeFromPoint();
        //sleep(1000);
        //robot.getIOSubsystem().runTransfer(true);
        //sleep(3000);
        //robot.getIOSubsystem().runTransfer(false);
    }
}
