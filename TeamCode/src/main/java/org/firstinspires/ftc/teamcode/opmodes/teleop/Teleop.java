package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode
{
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();

    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        robot.drive.setMaxSpeed(0.8);

        waitForStart();


        while (opModeIsActive()){
            time.reset();

            // update our gamepad extionsion
            gamepadEx1.update(gamepad1);

            // Actually drive the robot
            robot.drive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            // intake controls
            // Only allow intake when slides are not actively moving
            if (gamepad1.right_bumper
                    && robot.slides.getLiftState() == Slides.LIFT_STATE.HOLDING){
                robot.intake.intake();
            } else {
                robot.intake.stop();
            }

            // Right now height is constant down - this should change at some point.
            robot.intake.setHeight(robot.intake.DOWN);

            // Manual slides control - just up and down
            // We only allow the slides to move when the intake isn't moving - too much power
            if(robot.intake.getIntakeState() == Intake.IntakeState.STOP) {

                if (gamepad1.dpad_up && robot.slides.getPosition() < robot.slides.SLIDES_UP) {
                    robot.slides.slideUp();
                } else if (gamepad1.dpad_down && robot.slides.getPosition() > 0) {
                    robot.slides.slideDown();
                } else {
                    robot.slides.slideStop();
                }
            }

            // Arm controls
            if(gamepad1.a){
                robot.arm.up();
            } else if (gamepad1.b){
                robot.arm.down();
            }

            // End effector loop between states
            if(gamepadEx1.x.isNewlyReleased()){

                if(robot.arm.getPixelState() == Arm.PixelState.OPEN){
                    // lock the first pixel
                    robot.arm.lock1();
                } else if (robot.arm.getPixelState() == Arm.PixelState.ONE_LOCK){
                    // lock both pixels
                    robot.arm.fullLock();
                } else if (robot.arm.getPixelState() == Arm.PixelState.FULL_LOCK
                        && robot.arm.getArmState() == Arm.ArmState.ARM_UP){
                    // release the first pixel
                   robot.arm.release2();
                } else if (robot.arm.getPixelState() == Arm.PixelState.ONE_RELEASE
                && robot.arm.getArmState() == Arm.ArmState.ARM_UP){
                    // release the second pixel
                    robot.arm.fullRelease();
                }
            }

            // Show the speed to the user
            telemetry.addData("Max Speed:", robot.intake.getMaxSpeed());
            telemetry.addLine("Press A to increase and B to decrease");
            telemetry.addData("Slides pos",robot.slides.getPosition());
            telemetry.addData("PixelState", robot.arm.getPixelState());
            telemetry.addData("ArmState",robot.arm.getArmState());
            telemetry.addData("IntakeState",robot.intake.getIntakeState());
            telemetry.addData("Seconds in teleop ",time.seconds());
            telemetry.update();
        }

    }
}
