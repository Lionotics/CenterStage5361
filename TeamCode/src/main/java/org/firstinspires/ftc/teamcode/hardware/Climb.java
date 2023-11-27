package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Climb extends Mechanism {

    private DcMotor climb;
    private ServoImplEx hook;
    // These need tuning, random numbers for now
    private double HOOK_DOWN = 0.4;
    private double HOOK_UP = 0.5;
    private int CLIMB_UP = 500;
    public enum CLIMB_STATE {
        STOWED,
        RAISING,
        RAISED,
        CLIMBING,
        CLIMBED
    }
    private CLIMB_STATE climbState = CLIMB_STATE.STOWED;


    @Override
    public void init(HardwareMap hwMap) {
        climb = hwMap.dcMotor.get("climb");
        hook = (ServoImplEx)  hwMap.servo.get("hookTemp");

        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void hookUp() {
        hook.setPosition(HOOK_UP);
    }
    public void hookDown() {
        hook.setPosition(HOOK_DOWN);
    }
    public void disableHook(){hook.setPwmDisable();hook.setPwmEnable();}
    public int getPosition(){return climb.getCurrentPosition();}
    public void climbUp(){
        if (climb.getCurrentPosition() < CLIMB_UP) {
            climb.setPower(1);
        }
    }

    public void climbdown() {
        if (climb.getCurrentPosition() > 5) {
            climb.setPower(-1);
        }
    }
    public void stop(){
        climb.setPower(0);
    }


}

