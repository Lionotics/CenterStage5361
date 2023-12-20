package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Intake extends Mechanism {
    DcMotor intake;

     CRServo bottomRoller;
     Servo servoHeight, intakeFlap;
     public static double MAX_SPEED = 1;
     public  static double  UP = 0;
    public static  double DOWN = 0.22;
    public static double STACK = 0.05;
    public static double FLAP_OPEN = 0.2;
    public static double FLAP_CLOSED = 0.57;
    public enum IntakeState{
        IN,
        STOP,
        OUT
    }
    public enum IntakeHeight {
            DOWN,
            STACK
    }
    private IntakeState intakeState = IntakeState.STOP;
    public IntakeHeight intakeHeight = IntakeHeight.DOWN;

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.dcMotor.get("intake");
        bottomRoller = hwMap.crservo.get("intakeBottom");
        servoHeight = hwMap.servo.get("intakeHeight");
        intakeFlap = hwMap.servo.get("intakeFlap");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake(){
        intake.setPower(MAX_SPEED);
        bottomRoller.setPower(1);
        intakeState = IntakeState.IN;

    }
    public void stop(){
        intake.setPower(0);
        bottomRoller.setPower(0);
        intakeState = IntakeState.STOP;

    }
    public void outtake() {
        intake.setPower(-1);
        bottomRoller.setPower(-1);
        intakeState = IntakeState.OUT;

    }
    public void outtake(double power){
        intake.setPower(-power);
        bottomRoller.setPower(-1);
        intakeState = IntakeState.OUT;

    }
    public void setHeight(double height){
        servoHeight.setPosition(height);
    }
    public void intakeDown(){
        servoHeight.setPosition(DOWN);
        intakeHeight = IntakeHeight.DOWN;
        intakeFlap.setPosition(FLAP_CLOSED);
    }

    public void intakeStack(){
        servoHeight.setPosition(STACK);
        intakeHeight = IntakeHeight.STACK;
        intakeFlap.setPosition(FLAP_OPEN);

    }    public void setMaxSpeed(double speed){
        MAX_SPEED = speed;
    }
    public double getMaxSpeed(){
        return MAX_SPEED;

    }
    public IntakeState getIntakeState(){
        return intakeState;
    }
}
