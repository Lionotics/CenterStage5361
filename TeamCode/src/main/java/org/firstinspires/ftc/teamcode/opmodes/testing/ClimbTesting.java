package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Robot;


@TeleOp(group = "testers")
public class ClimbTesting extends LinearOpMode {

    Robot robot = new Robot(false);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.dpad_up){
                robot.climb.climbUp();
            } else if (gamepad1.dpad_down){
                robot.climb.climbdown();
            } else {
                robot.climb.stop();
            }

            if(gamepad1.a) {
                robot.climb.hookUp();
            } else if (gamepad1.b ){
                robot.climb.hookDown();
            } else if(gamepad1.y){
                robot.climb.disableHook();
            }

            telemetry.addData("Climb Pos", robot.climb.getPosition());
            telemetry.update();
        }
    }
}
