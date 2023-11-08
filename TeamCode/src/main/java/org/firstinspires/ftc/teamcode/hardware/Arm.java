package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism{

    public static double OFFSET = 0.006;
    public static double UP = 0.4;
    public static double DOWN = 0.21;
    public static double PIXEL1_IN = 0;
    public static double PIXEL2_IN = 0;
    public static double PIXEL1_OUT = 0;
    public static double PIXEL2_OUT = 0;

    private Servo arm1, arm2, pixel1, pixel2;
    @Override
    public void init(HardwareMap hwMap) {
        arm1 = hwMap.servo.get("pivotLeft");
        arm2 = hwMap.servo.get("pivotRight");
//        pixel1 = hwMap.servo.get("pixel1");
//        pixel2 = hwMap.servo.get("pixel2");
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
