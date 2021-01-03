package org.firstinspires.ftc.teamcode.team18103.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.src.Minibot;

@TeleOp
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
