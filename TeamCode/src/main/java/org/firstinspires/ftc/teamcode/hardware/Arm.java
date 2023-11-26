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

    public enum ArmState{
        ARM_UP,
        ARM_DOWN
    }
    public enum PixelState{
        OPEN,
        ONE_LOCK,
        FULL_LOCK,
        ONE_RELEASE
    }

    private Servo arm1, arm2, pixel1, pixel2;
    private ArmState armState;
    private PixelState pixelState;

    @Override
    public void init(HardwareMap hwMap) {
        arm1 = hwMap.servo.get("pivotLeft");
        arm2 = hwMap.servo.get("pivotRight");
        pixel1 = hwMap.servo.get("endLeft");
        pixel2 = hwMap.servo.get("endRight");
        pixel2.setDirection(Servo.Direction.REVERSE);
        down();
        fullRelease();
    }

    public void up(){
        arm1.setPosition(UP);
        arm2.setPosition(1 - UP - OFFSET);
        armState = ArmState.ARM_UP;
    }
    public void down(){
        arm1.setPosition(DOWN);
        arm2.setPosition(1 - DOWN - OFFSET);
        armState = ArmState.ARM_DOWN;
    }
    public void lock1(){
        pixel1.setPosition(PIXEL1_IN);
        pixelState = PixelState.ONE_LOCK;
    }
    public void lock2(){
        pixel2.setPosition(PIXEL2_IN);
    }
    public void release1(){
        pixel1.setPosition(PIXEL1_OUT);

    }
    public void release2(){
        pixel2.setPosition(PIXEL2_OUT);
        pixelState = PixelState.ONE_RELEASE;
    }

    public void fullRelease(){
        release1();
        release2();
        pixelState = PixelState.OPEN;
    }

    public void fullLock(){
        lock1();
        lock2();
        pixelState = PixelState.FULL_LOCK;
    }

    public PixelState getPixelState(){
        return pixelState;
    }

    public ArmState getArmState(){
        return armState;
    }

}
