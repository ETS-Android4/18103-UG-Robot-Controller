package org.firstinspires.ftc.teamcode.legacy.minibot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.legacy.minibot.Minibot;

@TeleOp
@Disabled
public class CryFinale extends OpMode {

    Minibot bot = new Minibot();

    @Override
    public void init() {

        bot.init(hardwareMap, telemetry);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        bot.loop(telemetry);

    }


}
