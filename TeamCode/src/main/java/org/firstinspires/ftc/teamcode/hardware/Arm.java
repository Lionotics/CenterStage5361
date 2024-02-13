package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Arm extends Mechanism{

    public static double OFFSET = 0.006;
    public static double UP = 0.81;
    public static double DOWN = 0.29;

    public static double PIXEL1_IN = 0.27;
    public static double PIXEL1_OUT = 0;

    public static double PIXEL2_IN = 0.2;
    public static double PIXEL2_OUT = 0;
    // if I need a delay to assume the arm is down, assume 1 second. (that's for sure enough)
    public enum ArmState{
        ARM_UP,
        ARM_DOWN,
        ARM_VERYDOWN,
        ARM_GROUND,
        ARM_UPMORE
    }
    public enum PixelState{
        OPEN,
        ONE_LOCK,
        FULL_LOCK,
        ONE_RELEASE
    }

    private ServoImplEx arm1, arm2, endGate, middleGate;
    public ArmState armState;
    private PixelState pixelState;

    @Override
    public void init(HardwareMap hwMap) {

        arm1 = (ServoImplEx) hwMap.servo.get("pivotLeft");
        arm2 = (ServoImplEx) hwMap.servo.get("pivotRight");
        endGate = (ServoImplEx) hwMap.servo.get("endGate");
        middleGate = (ServoImplEx) hwMap.servo.get("middleGate");
        middleGate.setDirection(Servo.Direction.REVERSE);

    }

    public void up(){
        arm1.setPosition(UP);
        arm2.setPosition(1 - UP - OFFSET);
        armState = ArmState.ARM_UP;
    }
    public void upMore(){
        arm1.setPosition(UP + 0.1);
        arm2.setPosition(1 - UP - OFFSET + 0.1);
        armState = ArmState.ARM_UPMORE;
    }

    public void down(){
        arm1.setPosition(DOWN);
        arm2.setPosition(1 - DOWN - OFFSET);
        armState = ArmState.ARM_DOWN;
    }
    public void ground(){
        // nothing as a placeholder
    }


    public void lock1(){
        endGate.setPosition(PIXEL1_IN);
        pixelState = PixelState.ONE_LOCK;
    }

    public void lock2(){
        middleGate.setPosition(PIXEL2_IN);
    }
    public void release1(){
        endGate.setPosition(PIXEL1_OUT);
        pixelState = PixelState.ONE_RELEASE;

    }

    public void release2(){
        middleGate.setPosition(PIXEL2_OUT);
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
