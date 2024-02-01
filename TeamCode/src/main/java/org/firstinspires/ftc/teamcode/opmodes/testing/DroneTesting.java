package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp()
public class DroneTesting extends LinearOpMode {
    Robot robot = new Robot(false);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){


            telemetry.addLine("A and B shoot and reset the drone servo");
            telemetry.addData("Airplane status", robot.airplane.hasShot());
            telemetry.update();

            if(gamepad1.a){
                robot.airplane.shootAirplane();
            } else if (gamepad1.b){
                robot.airplane.reset();
            }
        }

    }
}
