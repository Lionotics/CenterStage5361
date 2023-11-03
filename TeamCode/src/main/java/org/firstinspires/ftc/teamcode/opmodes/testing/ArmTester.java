package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class ArmTester extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.arm.up();


        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                robot.arm.up();
            } else if (gamepad1.b){
                robot.arm.down();
            }
        }

    }
}
