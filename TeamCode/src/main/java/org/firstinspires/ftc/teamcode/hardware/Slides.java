package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides extends Mechanism{
    DcMotor slideA, slideB;
    private double SLIDES_UP;
    @Override
    public void init(HardwareMap hwMap) {
        slideA = hwMap.dcMotor.get("slideATemp");
        slideB = hwMap.dcMotor.get("slideBTemp");

        slideA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideB.setDirection(DcMotorSimple.Direction.REVERSE);

        slideA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void slideUp(){
        if (slideA.getCurrentPosition() < SLIDES_UP) {
            slideA.setPower(1);
            slideB.setPower(1);
        }
    }
    public void slideDown(){
        if (slideA.getCurrentPosition() > 0) {
            slideA.setPower(-1);
            slideB.setPower(-1);
        }
    }
}
