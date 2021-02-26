package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;

import java.util.List;

import static org.firstinspires.ftc.teamcode.team18103.src.Constants.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.LABEL_SECOND_ELEMENT;

@Autonomous
public class IfYouDontWork extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        robot.getDriveSubsystem().getVisionProcessing().setZoom(2);
        AutoMode mode = AutoMode.None;
        /*
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

         */
        while (robot.getDriveSubsystem().getDataFusionTheta() > -30) {
            robot.getDriveSubsystem().setRotateMotors(-0.25);
            robot.loop(telemetry);
        }
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() < 1.5) {
            robot.getDriveSubsystem().setDriveMotors(0);
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


        while (robot.getDriveSubsystem().getDataFusionTheta() < 0) {
            //if(mode == AutoMode.None)
            //   mode = robot.getDriveSubsystem().getVisionProcessing().getAutoMode(); //I'm lazy and this is 200 IQ.
            robot.getDriveSubsystem().setRotateMotors(0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);

        while (robot.getDriveSubsystem().getDataFusionY() < 10) {
            //if(mode == AutoMode.None)
            //    mode = robot.getDriveSubsystem().getVisionProcessing().getAutoMode(); //I'm lazy and this is 200 IQ.
            robot.getDriveSubsystem().setDriveMotors(0.25);
            robot.loop(telemetry);
        }

        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(750);

        double currentPos = robot.getDriveSubsystem().getDataFusionY();

        if(mode == AutoMode.None) {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4 + 8) {
                robot.getDriveSubsystem().setDriveMotors(0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while (robot.getDriveSubsystem().getDataFusionTheta() < 40) {
                robot.getDriveSubsystem().setRotateMotors(0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);

            double currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) < currentDiagonalPosition + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);

            currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 4) {
                robot.getDriveSubsystem().setDriveMotors(-0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);

            while(robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);

            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 9) {
                robot.getDriveSubsystem().setDriveMotors(-0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while (robot.getDriveSubsystem().getDataFusionTheta() > -15) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 17) {
                robot.getDriveSubsystem().setDriveMotors(-0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while (robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + 25) {
                robot.getDriveSubsystem().setDriveMotors(0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while (robot.getDriveSubsystem().getDataFusionTheta() > -30) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            robot.getIOSubsystem().outtakeFromPoint3(3*24);
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
            while (robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 2) {
                robot.getDriveSubsystem().setDriveMotors(-0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);
            while(robot.getDriveSubsystem().getDataFusionTheta() > -30) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() < currentPos + 25) {
                robot.getDriveSubsystem().setDriveMotors(0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
        } else if(mode == AutoMode.One) {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4 + 8) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                if(Math.abs(robot.getDriveSubsystem().getDataFusionTheta()) > 5) {
                    if (0 - robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                        while(robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(200);
                    } else {
                        while(0 - robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(-0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(200);
                    }
                }
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while(robot.getDriveSubsystem().getDataFusionTheta() > -45) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            double currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) < currentDiagonalPosition + 6) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 8) {
                robot.getDriveSubsystem().setDriveMotors(-0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);
            while(robot.getDriveSubsystem().getDataFusionTheta() < 3) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);
            double currentPosY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() > currentPosY - 20) {
                robot.getDriveSubsystem().setDriveMotors(-0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            robot.loop(telemetry);
            sleep(200);

            while (robot.getDriveSubsystem().getDataFusionTheta() > -25) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);

            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);
            robot.getIOSubsystem().outtakeFromPoint3(3*24);
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);

            while (robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(200);
            currentPosY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() < currentPosY + 17) {
                robot.getDriveSubsystem().setDriveMotors(0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
        } else {
            double currentTheta = 0;
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4 + 8) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                if(Math.abs(robot.getDriveSubsystem().getDataFusionTheta() - currentTheta) > 5) {
                    if (currentTheta - robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                        while(robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    } else {
                        while(currentTheta - robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(-0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    }
                }
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while (robot.getDriveSubsystem().getDataFusionTheta() < 40) {
                robot.getDriveSubsystem().setRotateMotors(0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            double currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) < currentDiagonalPosition + 8) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 14) {
                robot.getDriveSubsystem().setDriveMotors(-0.40);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while(robot.getDriveSubsystem().getDataFusionTheta() > -30) {
                robot.getDriveSubsystem().setRotateMotors(-0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 27) {
                robot.getDriveSubsystem().setDriveMotors(-0.40);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(250);
            while(robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                robot.getDriveSubsystem().setRotateMotors(0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            double currentPositionY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() > currentPositionY - 15) {
                robot.getDriveSubsystem().setDriveMotors(-0.40);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            robot.loop(telemetry);
            sleep(500);

            while (robot.getDriveSubsystem().getDataFusionTheta() > -25) {
                robot.getDriveSubsystem().setRotateMotors(-0.27);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(300);
            robot.getIOSubsystem().outtakeFromPoint3(3*24);
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
            robot.getDriveSubsystem().setDriveMotors(0);

        }


    }
}