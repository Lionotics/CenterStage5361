package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Climb extends Mechanism {
    DcMotor climb;
    Servo hook;
    private double HOOK_DOWN;
    private double HOOK_UP;
    private double CLIMB_UP;


    @Override
    public void init(HardwareMap hwMap) {
        climb = hwMap.dcMotor.get("climb");
        hook = hwMap.servo.get("hookTemp");

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
}

