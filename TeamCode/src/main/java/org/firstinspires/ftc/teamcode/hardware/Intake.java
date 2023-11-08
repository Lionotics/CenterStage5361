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
     Servo servoHeight;
     public static double MAX_SPEED = 0.5;
     public  static double  UP = 0.1;
    public static  double DOWN = 0.54;

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.dcMotor.get("intake");
        bottomRoller = hwMap.crservo.get("intakeBottom");
        servoHeight = hwMap.servo.get("intakeHeight");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake(){
        intake.setPower(MAX_SPEED);
        bottomRoller.setPower(1);
    }
    public void stop(){
        intake.setPower(0);
        bottomRoller.setPower(0);
    }
    public void outtake() {
        intake.setPower(-1);
        bottomRoller.setPower(-1);
    }
    public void setHeight(double height){
        servoHeight.setPosition(height);
    }

    public void setMaxSpeed(double speed){
        MAX_SPEED = speed;
    }
    public double getMaxSpeed(){
        return MAX_SPEED;

    }
}
