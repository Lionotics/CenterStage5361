package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism{

    public static double OFFSET = 0.006;
    public static double UP = 0.4;
    public static double DOWN = 0.17;

    private Servo arm1, arm2;
    @Override
    public void init(HardwareMap hwMap) {
        arm1 = hwMap.servo.get("arm1");
        arm2 = hwMap.servo.get("arm2");

    }
    public void up(){
        arm1.setPosition(UP);
        arm2.setPosition(1 - UP - OFFSET);
    }
    public void down(){
        arm1.setPosition(DOWN);
        arm2.setPosition(1 - DOWN - OFFSET);
    }
}
