package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Author: Akhil G
 */

public abstract class Subsystem {

    public abstract void init(HardwareMap ahMap);

    public abstract void start();

    public abstract void update();

    //public abstract StateMachine getStateMachine();

}
