package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Intake extends Mechanism {
    DcMotor intake;

     Servo servoHeight, intakeFlap;

     public static double MAX_SPEED = 1;
     // UP is all the way up for getting out of the way
     public  static double  UP = 0.11;
     // DOWN is all the way down for intaking from the ground
    public static  double DOWN = 0.88;
    // Individual stack heights
    public static double STACK_5 = 0.69;
    public static double STACK_4 = 0;
    public static double STACK_3 = 0;

    public static double FLAP_DOWN = 0.33;
    public static double FLAP_UP = 0.74;

    public enum IntakeState{
        IN,
        STOP,
        OUT
    }

    public enum IntakeHeight {
        UP,
        DOWN,
        STACK_5,
        STACK_4,
        STACK_3,
    }

    private IntakeState intakeState = IntakeState.STOP;
    public IntakeHeight intakeHeight = IntakeHeight.DOWN;

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.dcMotor.get("intake");
        servoHeight = hwMap.servo.get("intakeHeight");
        intakeFlap = hwMap.servo.get("intakeFlap");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // In / Out / Stop
    public void intake(){
        intake.setPower(MAX_SPEED);
        intakeState = IntakeState.IN;

    }
    public void stop(){
        intake.setPower(0);
        intakeState = IntakeState.STOP;

    }
    public void outtake() {
        intake.setPower(-1);
        intakeState = IntakeState.OUT;

    }

    // Height controls
    // This function should get removed and replaced


    public void intakeDown(){
        stackHeight(0);
    }

    public void intakeUp(){
        servoHeight.setPosition(UP);
        intakeHeight = IntakeHeight.UP;

    }
    public void stackHeight(int pos){
        switch (pos){
            case 0:
                servoHeight.setPosition(DOWN);
                intakeHeight = IntakeHeight.DOWN;
                intakeFlap.setPosition(FLAP_UP);
                break;
            case 3:
                servoHeight.setPosition(STACK_3);
                intakeHeight = IntakeHeight.STACK_3;
                intakeFlap.setPosition(FLAP_DOWN);
                break;
            case 4:
                servoHeight.setPosition(STACK_4);
                intakeHeight = IntakeHeight.STACK_4;
                intakeFlap.setPosition(FLAP_DOWN);

                break;
            case 5:
                servoHeight.setPosition(STACK_5);
                intakeHeight = IntakeHeight.STACK_5;
                intakeFlap.setPosition(FLAP_DOWN);
                break;
        }
        // Do something with the flap here maybe?
    }

    public void setMaxSpeed(double speed){
        MAX_SPEED = speed;
    }
    public double getMaxSpeed(){
        return MAX_SPEED;

    }

    public IntakeState getIntakeState(){
        return intakeState;
    }
}
