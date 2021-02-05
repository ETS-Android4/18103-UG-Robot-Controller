package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble extends Subsystem {
    Servo latch;
    Servo joint;

    @Override
    public void init(HardwareMap ahMap) {
        latch = ahMap.get(Servo.class, "joint");
        joint = ahMap.get(Servo.class, "latch");
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void moveJoint(boolean action) {
        if(action) {
            joint.setPosition(joint.getPosition() + 0.05);
        } else {
            joint.setPosition(joint.getPosition() - 0.02);
        }
    }

    /**
     * @param action if -1, no input detected. if 0, close. if 1, open
     */
    public void moveLatch(double action) {
        if(action != -1) {
            latch.setPosition(action);
        }
    }
}
