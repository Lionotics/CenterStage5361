package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp(group = "testers")
public class IntakeTesting extends LinearOpMode {
    private Robot robot = new Robot(false);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                robot.intake.intake();
            } else {
                robot.intake.stop();
            }
            if (gamepad1.dpad_up) {
              robot.intake.setHeight(robot.intake.UP);
            } else if (gamepad1.dpad_down) {
                robot.intake.setHeight(robot.intake.DOWN);
            }
        }
    }
}
