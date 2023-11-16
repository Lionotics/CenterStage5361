package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode
{
//    Robot robot = new Robot();
//    Drivetrain drive = new Drivetrain();
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.drive.setMaxSpeed(0.8);

        waitForStart();


        while (opModeIsActive()){

            // update our gamepad
            gamepadEx1.update(gamepad1);

            // Actually drive the robot
            robot.drive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            if (gamepad1.right_bumper){
                robot.intake.intake();
            } else {
                robot.intake.stop();
            }
            robot.intake.setHeight(robot.intake.DOWN);

            if (gamepad1.dpad_up && robot.slides.getPosition() < robot.slides.SLIDES_UP) {
                robot.slides.slideUp();
            } else if (gamepad1.dpad_down && robot.slides.getPosition() > 0) {
                robot.slides.slideDown();
            } else {
                robot.slides.slideStop();
            }

            if(gamepad1.a){
                robot.arm.up();
            } else if (gamepad1.b){
                robot.arm.down();
            }

            if(gamepad1.x){
                robot.arm.lock1();
            } else {
                robot.arm.release1();
            }

            if(gamepad1.y){
                robot.arm.lock2();
            } else{
                robot.arm.release2();
            }


            // Show the speed to the user
            telemetry.addData("Max Speed:", robot.intake.getMaxSpeed());
            telemetry.addLine("Press A to increase and B to decrease");
            telemetry.addData("Slides pos",robot.slides.getPosition());

            telemetry.update();
        }

    }
}
