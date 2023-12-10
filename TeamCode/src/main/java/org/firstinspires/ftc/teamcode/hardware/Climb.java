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
    public enum ClimbState {
        STOWED,
        RAISING,
        RAISED,
        CLIMBING,
        CLIMBED
    }
    private ClimbState climbState = ClimbState.STOWED;


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
        climbState = ClimbState.RAISING;
    }
    public void startClimb(){
        climbState = ClimbState.CLIMBING;
    }
    public ClimbState getClimbState(){
        return climbState;

    }
    public void updateClimb(){
        switch (climbState){
            case RAISING:
                climbUp();
                if(this.getPosition() >= CLIMB_UP - 10){
                    stop();
                    hookUp();
                    climbState = ClimbState.RAISED;
                }
                break;
            case RAISED:
                stop();
                break;
            case CLIMBING:
                hook.setPwmDisable();
                climbDown();
                if(this.getPosition() < 10){
                    climbState = ClimbState.CLIMBED;
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

