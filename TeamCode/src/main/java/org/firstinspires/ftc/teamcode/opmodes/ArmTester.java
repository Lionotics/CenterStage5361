package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Arm;
@TeleOp
public class ArmTester extends LinearOpMode {
    Arm arm = new Arm();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        arm.up();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                arm.up();
            } else if (gamepad1.b){
                arm.down();
            }
        }

    }
}
