package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

import java.util.List;

import static org.firstinspires.ftc.teamcode.team18103.src.Constants.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.LABEL_SECOND_ELEMENT;

@Autonomous
public class DriveNShootRedAuto extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        robot.getDriveSubsystem().getVisionProcessing().setZoom(2);
        AutoMode mode = AutoMode.None;
        while(!isStarted()) {
            List<Recognition> searches = robot.getDriveSubsystem().getVisionProcessing().search();
            if (searches != null) {
                for (Recognition i : searches) {
                    if (i.getLabel().equals(
                            LABEL_FIRST_ELEMENT)) {
                        mode = AutoMode.Four;
                    } else if (i.getLabel().equals(
                            LABEL_SECOND_ELEMENT)) {
                        mode = AutoMode.One;
                    }
                }
            }

            telemetry.addData("Real Vision", mode);
            telemetry.update();
        }
        waitForStart();
        robot.start();

        if (mode == AutoMode.One) {
            while (Math.abs(95 - robot.getDriveSubsystem().getDataFusionY()) > 15) {
                robot.getDriveSubsystem().POVMecanumDrive(-1.2 + (robot.getDriveSubsystem().getDataFusionY())/95, 0, 0, DriveMode.Balanced);
                robot.loop(telemetry);
            }

            robot.getDriveSubsystem().setDriveMotors(0);

            while (robot.getDriveSubsystem().getDataFusionTheta() < 45) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);

            robot.getDriveSubsystem().setDriveMotors(0.25);
            sleep(750);
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            robot.getDriveSubsystem().setDriveMotors(-0.25);
            sleep(750);
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);

            while (robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);

            while (Math.abs(55 - robot.getDriveSubsystem().getDataFusionY()) > 5) {
                robot.getDriveSubsystem().POVMecanumDrive(-0.9 + (robot.getDriveSubsystem().getDataFusionY())/55, 0, 0, DriveMode.Balanced);
                robot.loop(telemetry);
            }

            robot.getDriveSubsystem().setDriveMotors(0);

        } else {
            while (Math.abs(75 - robot.getDriveSubsystem().getDataFusionY()) > 15) {
                robot.getDriveSubsystem().POVMecanumDrive(-1.25 + (robot.getDriveSubsystem().getDataFusionY())/75, 0, 0, DriveMode.Balanced);
                robot.loop(telemetry);
            }

            robot.getDriveSubsystem().setDriveMotors(0);
        }

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
                    82 - robot.getDriveSubsystem().getDataFusionY(), //108
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            //robot.getDriveSubsystem().setRotateMotors(-0.18);
        }
        while (timer.seconds() < 1.5) {
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    82 - robot.getDriveSubsystem().getDataFusionY(), // 108
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            robot.getDriveSubsystem().setRotateMotors(-0.18);
        }
        while (timer.seconds() < 5) {
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    60 - robot.getDriveSubsystem().getDataFusionY(), // 108
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            robot.getIOSubsystem().runTransfer(true);
            robot.getDriveSubsystem().setRotateMotors(-0.2);
        }

        robot.getIOSubsystem().runTransfer(false);
        robot.getIOSubsystem().runOuttake(false);
        robot.getDriveSubsystem().setDriveMotors(0);

        while (robot.getDriveSubsystem().getDataFusionTheta() < 0) {
            robot.getDriveSubsystem().setRotateMotors(0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);

        while (Math.abs(75 - robot.getDriveSubsystem().getDataFusionY()) > 2.5) {
            robot.getDriveSubsystem().POVMecanumDrive(-1.60 + (robot.getDriveSubsystem().getDataFusionY())/55, 0, 0, DriveMode.Balanced);
            robot.loop(telemetry);
        }

        //robot.getIOSubsystem().OuttakeFromPoint();
        //sleep(1000);
        //robot.getIOSubsystem().runTransfer(true);
        //sleep(3000);
        //robot.getIOSubsystem().runTransfer(false);
    }
}
