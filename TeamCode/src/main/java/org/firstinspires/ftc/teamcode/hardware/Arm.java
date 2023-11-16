package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism{

    public static double OFFSET = 0.006;
    public static double UP = 0.88;
    public static double DOWN = 0.21;
    public static double PIXEL1_IN = 0.27;
    public static double PIXEL2_IN = 0.4;
    public static double PIXEL1_OUT = 0;
    public static double PIXEL2_OUT = 0.1;

    private Servo arm1, arm2, pixel1, pixel2;
    @Override
    public void init(HardwareMap hwMap) {
        arm1 = hwMap.servo.get("pivotLeft");
        arm2 = hwMap.servo.get("pivotRight");
        pixel1 = hwMap.servo.get("endLeft");
        pixel2 = hwMap.servo.get("endRight");
        pixel2.setDirection(Servo.Direction.REVERSE);
    }
    public void up(){
        arm1.setPosition(UP);
        arm2.setPosition(1 - UP - OFFSET);
        // lock both pixel servos?
    }
    public void down(){
        arm1.setPosition(DOWN);
        arm2.setPosition(1 - DOWN - OFFSET);
        // release the pixel servos
    }
    public void lock1(){
        pixel1.setPosition(PIXEL1_IN);
    }
    public void lock2(){
        pixel2.setPosition(PIXEL2_IN);
    }
    public void release1(){
        pixel1.setPosition(PIXEL1_OUT);
    }
    public void release2(){
        pixel2.setPosition(PIXEL2_OUT);
    }
    public void fullRelease(){
        release1();
        release2();
    }
    public void fullLock(){
        lock1();
        lock2();
    }
}
