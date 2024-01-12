package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Arm extends Mechanism{

    public static double OFFSET = 0.006;
    public static double UP = 0.65;
    public static double DOWN = 0.19;
    public static double VERY_DOWN = 0.12;
    public static double GROUND = 0.89;
    public static double PIXEL1_IN = 0.27;
    public static double PIXEL2_IN = 0.6;
    public static double PIXEL1_OUT = 0;
    public static double PIXEL2_OUT = 0.2;

    public enum ArmState{
        ARM_UP,
        ARM_DOWN,
        ARM_VERYDOWN,
        ARM_GROUND
    }
    public enum PixelState{
        OPEN,
        ONE_LOCK,
        FULL_LOCK,
        ONE_RELEASE
    }

    private ServoImplEx arm1, arm2, pixel1, pixel2;
    public ArmState armState;
    private PixelState pixelState;

    @Override
    public void init(HardwareMap hwMap) {
        arm1 = (ServoImplEx) hwMap.servo.get("pivotLeft");
        arm2 = (ServoImplEx) hwMap.servo.get("pivotRight");
        pixel1 = (ServoImplEx) hwMap.servo.get("endLeft");
        pixel2 = (ServoImplEx) hwMap.servo.get("endRight");
        pixel2.setDirection(Servo.Direction.REVERSE);
//        arm1.setPwmRange(new PwmControl.PwmRange(500,2500));
//        arm2.setPwmRange(new PwmControl.PwmRange(500,2500));


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
    public void veryDown(){
        arm1.setPosition(VERY_DOWN);
        arm2.setPosition(1-VERY_DOWN-OFFSET);
        armState = ArmState.ARM_VERYDOWN;
    }
    public void ground(){
        arm1.setPosition(GROUND);
        arm2.setPosition(1 - GROUND);
        armState = ArmState.ARM_GROUND;
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
