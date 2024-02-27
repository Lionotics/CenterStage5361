package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Slides extends Mechanism{

    private DcMotorEx slideA, slideB;
    public static int SLIDES_UP = 2200;
    public static double SLIDES_HOLD = 0.05; // needs redoing for new motors
    public static double MAX_SPEED = 1;
    public static double MAX_AUTO_SPEED = 1;
    public static int SLIDES_AUTO = 350; // needs redoing for new motors
    public static int TRANSITION_POINT = 750; // needs redoing for new motors
    public static int CLIMB_UP = 0;
    // min for scoring ~300

    // PID Loop
    private PIDController controller;
    public static int target = 0;
    public static double Kg = SLIDES_HOLD;
    public static double Kp = 0.005;
    public static double Ki = 0;
    public static double Kd = 0.0;

    // state machine
    public enum LIFT_STATE {
        AUTO_MOVE,
        MANUAL_UP,
        MANUAL_DOWN,
        HOLDING
    }

    private LIFT_STATE liftState = LIFT_STATE.HOLDING;

    @Override
    public void init(HardwareMap hwMap) {
        slideA = (DcMotorEx) hwMap.dcMotor.get("slidesRight");
        slideB = (DcMotorEx) hwMap.dcMotor.get("slidesLeft");

        slideA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideB.setDirection(DcMotorSimple.Direction.REVERSE);

        slideA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(Kp,Ki,Kd);
        controller.setPID(Kp,Ki,Kd);

        target = 0;


    }
    public void autoMoveTo(int newTarget){

        setTarget(newTarget);
        liftState = LIFT_STATE.AUTO_MOVE;

    }

    public void pidLoop(){
        // Here for testing. get rid of it after
        controller.setPID(Kp,Ki,Kd);

        double pos = this.getPosition();

        double power = controller.calculate(pos,target);
        if(this.getPosition() > 10) { // only add feedforward when not all the way down
            slideA.setPower((power * MAX_AUTO_SPEED) + Kg);
            slideB.setPower((power * MAX_AUTO_SPEED) + Kg);
        } else {
            slideA.setPower((power * MAX_AUTO_SPEED));
            slideB.setPower((power * MAX_AUTO_SPEED));
        }

    }
    public void setTarget(int target){
        Slides.target = target;}
    public int getTarget(){return target;}

    // Manual movement
    public void slideUp(){
        if (slideA.getCurrentPosition() < SLIDES_UP) {
            slideA.setPower(MAX_SPEED);
            slideB.setPower(MAX_SPEED);
        }
    }
    public void manualUp(){
        liftState = LIFT_STATE.MANUAL_UP;
    }

    public void slideDown(){
        if (slideA.getCurrentPosition() > 0) {
            slideA.setPower(-MAX_SPEED);
            slideB.setPower(-MAX_SPEED);
        }
    }
    public void manualDown(){
        liftState = LIFT_STATE.MANUAL_DOWN;
    }

    public void slideStop(){
        if(this.getPosition() > 10) {
            slideA.setPower(SLIDES_HOLD);
            slideB.setPower(SLIDES_HOLD);
        } else {
            slideA.setPower(0);
            slideB.setPower(0);

        }
    }
    public void hold(){
        liftState = LIFT_STATE.HOLDING;

    }

    public void loop(){
        switch (liftState){
            case AUTO_MOVE:
                pidLoop();
                break;
            case MANUAL_DOWN:
                slideDown();
                break;
            case MANUAL_UP:
                slideUp();
                break;
            case HOLDING:
                slideStop();
                break;
        }

    }
    public int getPosition(){
        return slideA.getCurrentPosition();
    }

    public LIFT_STATE getLiftState(){
        return liftState;
    }


}
