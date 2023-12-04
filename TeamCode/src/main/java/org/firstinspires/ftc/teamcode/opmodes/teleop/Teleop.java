package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Climb;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();

    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        robot.drive.setMaxSpeed(0.8);

        waitForStart();
        robot.intake.intakeDown();



        while (opModeIsActive()) {
            time.reset();

            // update our gamepad extionsion
            gamepadEx1.update(gamepad1);

            // Actually drive the robot
            robot.drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // intake controls
            // Only allow intake when slides are not actively moving
            if (gamepad1.right_bumper
                    && robot.slides.getLiftState() == Slides.LIFT_STATE.HOLDING) {
                robot.intake.intake();
            } else if (gamepad1.left_trigger > 0.6) {
                robot.intake.outtake();
            }
            else{
                robot.intake.stop();
            }

            // Right now height is constant down - this should change at some point.
            if(gamepadEx1.y.isNewlyPressed()) {
                if (robot.intake.intakeHeight == Intake.IntakeHeight.STACK){
                    robot.intake.intakeDown();
                } else if (robot.intake.intakeHeight == Intake.IntakeHeight.DOWN){
                    robot.intake.intakeStack();
                }
            }

            // Manual slides control - just up and down
            // We only allow the slides to move when the intake isn't moving - too much power
            if (robot.intake.getIntakeState() == Intake.IntakeState.STOP) {

                if ((gamepad1.b || gamepad1.dpad_up) && robot.slides.getPosition() < Slides.SLIDES_UP) {
                    // Test this carefully - it's sketchy 100% of the way
                    if(gamepadEx1.b.isNewlyPressed() && robot.slides.getPosition() < Slides.TRANSITION_POINT){
                        // if slides are down and B is newly pressed, do a full auto deploy for scoring
                        robot.arm.fullLock();
                        robot.arm.up();
                        robot.slides.autoMoveTo(Slides.TRANSITION_POINT);
                    } else if((robot.slides.getPosition() > Slides.TRANSITION_POINT - 10 && gamepad1.b) || gamepad1.dpad_up){
                        // B is pressed, and slides have already gone auto up to the first point
                        robot.slides.manualUp();
                    }

                } else if (gamepad1.left_bumper && robot.slides.getPosition() > 0) {
                    robot.slides.manualDown();
                } else if(robot.slides.getLiftState() != Slides.LIFT_STATE.AUTO_MOVE) {
                    robot.slides.hold();
                }

            }

            // All of the above functions just change the state of the slides. The loop function actually moves them.
            robot.slides.loop();


            // Arm controls
            if (gamepadEx1.a.isNewlyPressed()) {
                if(robot.arm.armState == Arm.ArmState.ARM_DOWN) {
                    robot.arm.up();

                } else if (robot.arm.armState == Arm.ArmState.ARM_UP){
                    robot.arm.down();
                    robot.arm.fullRelease();
                }

            }

            // End effector loop between states
            if (gamepadEx1.x.isNewlyReleased()) {

                if (robot.arm.getPixelState() == Arm.PixelState.OPEN) {
                    // lock the pixels
                    robot.arm.fullLock();
                } else if (robot.arm.getPixelState() == Arm.PixelState.ONE_LOCK) {
                    // lock both pixels
                    robot.arm.fullLock();
                } else if (robot.arm.getPixelState() == Arm.PixelState.FULL_LOCK
                        && robot.arm.getArmState() == Arm.ArmState.ARM_UP) {
                    // release the first pixel
                    robot.arm.release2();
                } else if (robot.arm.getPixelState() == Arm.PixelState.ONE_RELEASE
                        && robot.arm.getArmState() == Arm.ArmState.ARM_UP) {
                    // release the second pixel
                    robot.arm.fullRelease();
                }
            }


            if (gamepadEx1.left.isNewlyPressed()) {
                if(robot.climb.getClimbState() == Climb.CLIMB_STATE.STOWED){
                    robot.climb.startRaise();
                } else if (robot.climb.getClimbState() == Climb.CLIMB_STATE.RAISED){
                    robot.climb.startClimb();
                }
            }

            robot.climb.updateClimb();

            // Airplane
            if(gamepad1.dpad_right){
                robot.airplane.shootAirplane();
            }

            // Telemetry
            telemetry.addData("Slides pos", robot.slides.getPosition());
            telemetry.addData("PixelState", robot.arm.getPixelState());
            telemetry.addData("ArmState", robot.arm.getArmState());
            telemetry.addData("IntakeState", robot.intake.getIntakeState());
            telemetry.addData("Climb pos", robot.climb.getPosition());
            telemetry.addData("Climb State ", robot.climb.getClimbState());
            telemetry.addData("Intake Height", robot.intake.intakeHeight);
            telemetry.addData("Slides state",robot.slides.getLiftState());
            telemetry.update();
        }

    }

}