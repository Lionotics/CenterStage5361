package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends Mechanism {
    DcMotor intake;

     CRServo bottomRoller;


    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.dcMotor.get("intake");
        bottomRoller = hwMap.crservo.get("BottomRoller");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(){
        intake.setPower(1);
        bottomRoller.setPower(1);
    }
    public void outtake() {
        intake.setPower(-1);
        bottomRoller.setPower(-1);
    }
}
