package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime elapsedTime = new ElapsedTime();

    private boolean inEndgame = false;

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        robot.drive.setMaxSpeed(1);

        waitForStart();
        robot.arm.down();
        robot.arm.fullRelease();
        robot.arm.lock1();
        robot.intake.intakeDown();
        timer.reset();



        while (opModeIsActive()) {

            // update our gamepad extension
            gamepadEx1.update(gamepad1);

            // Actually drive the robot
            robot.drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            // Add variable speed based on distance to backboard here

            // intake controls
            // Only allow intake when slides are down
            if (gamepad1.right_bumper
                    && robot.slides.getPosition() < 10) {

                robot.intake.intake();

                // when intaking, disable arm to make the transfer work better
                robot.arm.disableArm();

            } else if (gamepad1.left_trigger > 0.6) {
                robot.intake.outtake();
            } else {
                robot.intake.stop();
            }

            // Height toggle between all the way down and all the way up
            if(gamepadEx1.y.isNewlyPressed()) {
                if (robot.intake.intakeHeight == Intake.IntakeHeight.DOWN){
                    robot.intake.stackHeight(5);
                } else {
                    robot.intake.intakeDown();
                }
            }

            // restarting the intake when up should move it down an increment
            if(gamepadEx1.rightBumper.isNewlyReleased()){

                if(robot.intake.intakeHeight == Intake.IntakeHeight.STACK_5){
                    robot.intake.stackHeight(4);
                } else if (robot.intake.intakeHeight == Intake.IntakeHeight.STACK_4){
                    robot.intake.stackHeight(3);
                }

            }

            // Slides control - partial automatic, partial manual
            // We only allow the slides to move when the intake isn't moving - too much power
            if (robot.intake.getIntakeState() == Intake.IntakeState.STOP) {

                if ((gamepad1.b || gamepad1.dpad_up) && robot.slides.getPosition() < Slides.SLIDES_UP) {

                    if(gamepadEx1.b.isNewlyPressed() && robot.slides.getPosition() < Slides.TRANSITION_POINT){
                        // if slides are down and B is newly pressed, do a full auto deploy for scoring
                        robot.arm.up();
                        // bypass the second lock stage if the drive doesn't want to
                        if(robot.arm.getPixelState() == Arm.PixelState.ONE_LOCK){
                            robot.arm.release2();
                        }

                        robot.slides.autoMoveTo(Slides.TRANSITION_POINT);
                    } else if((robot.slides.getPosition() > Slides.TRANSITION_POINT - 10 && gamepad1.b) || gamepad1.dpad_up){
                        // B is pressed, and slides have already gone auto up to the first point
                        robot.slides.manualUp();
                        robot.arm.enableArm();
                    }

                } else if (gamepad1.left_bumper && robot.slides.getPosition() > 0) {
                    robot.slides.manualDown();
                }  else if (gamepad1.dpad_down){
                    robot.slides.forceDown();
                }else if(robot.slides.getLiftState() != Slides.LiftState.AUTO_MOVE) {
                    robot.slides.hold();
                }

            }

            // All of the above functions just change the state of the slides. The loop function actually moves them.
            robot.slides.loop();


            // Arm controls
            if (gamepadEx1.a.isNewlyPressed()) {
                robot.arm.enableArm();

                if(robot.arm.armState == Arm.ArmState.ARM_DOWN) {

                    robot.arm.up();
                    // bypass the second lock stage if the drive doesn't want to
                    if(robot.arm.getPixelState() == Arm.PixelState.ONE_LOCK){
                        robot.arm.release2();
                    }

                } else if (robot.arm.armState == Arm.ArmState.ARM_UP){
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.arm.lock1();
                }

            }

            // End effector loop between states
            if (gamepadEx1.x.isNewlyPressed()) {

                if(robot.arm.getArmState() == Arm.ArmState.ARM_DOWN){
                    if (robot.arm.getPixelState() == Arm.PixelState.ONE_LOCK) {
                        // lock both pixels
                        robot.arm.fullLock();
                    } else if (robot.arm.getPixelState() ==  Arm.PixelState.FULL_LOCK){
                        robot.arm.unlock2();
                    }
                } else if (robot.arm.getArmState() == Arm.ArmState.ARM_UP) {
                // arm is up
                    if (robot.arm.getPixelState() == Arm.PixelState.FULL_LOCK) {
                        // release the first pixel if both are locked
                        robot.arm.release1();
                    } else if (robot.arm.getPixelState() == Arm.PixelState.ONE_RELEASE) {
                        // release the second pixel, or both if the second was not locked
                        robot.arm.fullRelease();
                        // at this pont we are done, add a new state for resetting everything
                    } else if (robot.arm.getPixelState() == Arm.PixelState.OPEN){
                        // at this pont we are done, add a new state for resetting everything
                        robot.arm.down();
                        // slides down
                        robot.slides.autoMoveTo(0);
                        robot.arm.fullRelease();
                        robot.arm.lock1();

                    }
                }
            }

            // Put Climb Here
            if(gamepadEx1.left.isNewlyReleased()){
                robot.slides.updateClimb();
                if(robot.slides.getClimbState() == Slides.ClimbState.MOVING_UP){
                    robot.arm.up();

                } else if (robot.slides.getClimbState() == Slides.ClimbState.MOVING_DOWN){
                    robot.intake.intakeUp();

                }
            }


            // Airplane
            if(gamepad1.dpad_right){
                robot.airplane.shootAirplane();
            }

            if(timer.seconds() >= 90){
                inEndgame = true;
            }

            // Telemetry
            telemetry.addData("Slides pos", robot.slides.getPosition());
            telemetry.addData("PixelState", robot.arm.getPixelState());
            telemetry.addData("ArmState", robot.arm.getArmState());
            telemetry.addData("IntakeState", robot.intake.getIntakeState());
            telemetry.addData("Intake Height", robot.intake.intakeHeight);
            telemetry.addData("Slides state",robot.slides.getLiftState());
            telemetry.addData("Time in teleop", timer.seconds());
            telemetry.addData("In endgame", inEndgame);
            telemetry.addData("Loop Times", elapsedTime.milliseconds());
            telemetry.update();

            // tracking loop times
            elapsedTime.reset();

        }

    }

}