package org.firstinspires.ftc.teamcode.testChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class Test_Chassis extends LinearOpMode {
    IMU imu = new IMU();
    Drive robot = new Drive(imu);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        imu.init(hardwareMap);
        telemetry.addData("Init", true);
        waitForStart();
        robot.distanceddrive(2);
    }


}
