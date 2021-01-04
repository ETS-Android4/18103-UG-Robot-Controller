package org.firstinspires.ftc.teamcode.legacy.minibot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Author: Akhil G
 */

public class MinibotSubsystem {

    DcMotorEx motor;

    public MinibotSubsystem() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void start() {

    }

    public void loop(Telemetry telemetry) {

    }

    public void Telemetry(Telemetry telemetry) {

    }

    public DcMotorEx getMotor() {
        return motor;
    }
}
