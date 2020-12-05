package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble extends Subsystem {
    Servo latch;
    CRServo joint;

    public Wobble() {}

    @Override
    public void init(HardwareMap ahMap) {
        latch = ahMap.get(Servo.class, "latch");
        joint = ahMap.get(CRServo.class, "joint");
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void moveJoint(double power) {
        joint.setPower(power);
    }

    /**
     * @param action if -1, no input detected. if 0, close. if 1, open
     */
    public void moveLatch(int action) {
        if(action != -1) {
            latch.setPosition(action);
        }
    }
}
