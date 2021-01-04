package org.firstinspires.ftc.teamcode.legacy.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Author: Akhil G
 */

public class Minibot {

    MinibotSubsystem bot = new MinibotSubsystem();

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
