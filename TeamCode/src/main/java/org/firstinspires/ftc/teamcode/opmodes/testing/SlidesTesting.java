package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp()
public class SlidesTesting extends LinearOpMode {
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a && robot.slides.getPosition() < robot.slides.SLIDES_UP) {
                robot.slides.slideUp();
            } else if (gamepad1.b && robot.slides.getPosition() > 0) {
                robot.slides.slideDown();
            } else {
                robot.slides.slideStop();
            }


            telemetry.addData("Position", robot.slides.getPosition());
            telemetry.update();
            }
        }
}

