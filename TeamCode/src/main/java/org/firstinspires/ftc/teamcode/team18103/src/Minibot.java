package org.firstinspires.ftc.teamcode.team18103.src;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Author: Akhil G
 */

public class Minibot {

    org.firstinspires.ftc.teamcode.team18103.subsystems.Minibot bot = new org.firstinspires.ftc.teamcode.team18103.subsystems.Minibot();

    public Minibot() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        bot.init(hardwareMap, telemetry);

    }

    public void start() {

    }

    public void loop(Telemetry telemetry) {

        telemetry.addData("Motor Position: ", bot.getMotor().getCurrentPosition());

    }

    public void Telemetry(Telemetry telemetry) {

    }

}
