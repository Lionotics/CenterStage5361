package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Climb extends Mechanism {

    private DcMotor climb;
    private ServoImplEx hook;
    // These need tuning, random numbers for now
    public static double HOOK_DOWN = 1;
    public static double HOOK_UP = 0.34;
    public static int CLIMB_UP = 8700;
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
        hook = (ServoImplEx)  hwMap.servo.get("hook");

        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        hookDown();
    }

    public void startRaise(){
        climbState = CLIMB_STATE.RAISING;
    }
    public void startClimb(){
        climbState = CLIMB_STATE.CLIMBING;
    }
    public CLIMB_STATE getClimbState(){
        return climbState;

    }
    public void updateClimb(){
        switch (climbState){
            case RAISING:
                climbUp();
                if(this.getPosition() >= CLIMB_UP - 10){
                    stop();
                    hookUp();
                    climbState = CLIMB_STATE.RAISED;
                }
                break;
            case RAISED:
                stop();
                break;
            case CLIMBING:
                hook.setPwmDisable();
                climbDown();
                if(this.getPosition() < 10){
                    climbState = CLIMB_STATE.CLIMBED;
                    stop();
                }
                break;
            case CLIMBED:
                stop();
                break;
        }
    }
    public void hookUp() {
        hook.setPosition(HOOK_UP);

    }
    public void hookDown() {
        hook.setPosition(HOOK_DOWN);
    }
    public int getPosition(){return climb.getCurrentPosition();}
    public void climbUp(){
        if (climb.getCurrentPosition() < CLIMB_UP) {
            climb.setPower(1);
        }
    }
    public boolean enabled(){
        return hook.isPwmEnabled();
    }

    public void climbDown() {
        if (climb.getCurrentPosition() > 5) {
            climb.setPower(-1);
        }

    }
    public void manualUp() {
        climb.setPower(1);
    }
    public void manualDown(){
        climb.setPower(-1);

    }
        public void stop(){
        climb.setPower(0);
    }


}

