package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Slides extends Mechanism{
    DcMotor slideA, slideB;
    public static double SLIDES_UP = 960;
    public static double SLIDES_HOLD = 0.008;
    @Override
    public void init(HardwareMap hwMap) {
        slideA = hwMap.dcMotor.get("slidesRight");
        slideB = hwMap.dcMotor.get("slidesLeft");

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
            slideA.setPower(0.4);
            slideB.setPower(0.4);
        }
    }
    public void slideDown(){
        if (slideA.getCurrentPosition() > 0) {
            slideA.setPower(-0.4);
            slideB.setPower(-0.4);
        }
    }
    public void slideStop(){
        slideA.setPower(SLIDES_HOLD);
        slideB.setPower(SLIDES_HOLD);
    }
    public int getPosition(){
        return slideA.getCurrentPosition();
    }
}
