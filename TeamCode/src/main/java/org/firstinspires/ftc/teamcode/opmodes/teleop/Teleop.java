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

        waitForStart();

        robot.drive.setMaxSpeed(0.5);

        while (opModeIsActive()){

            // update our gamepad
            gamepadEx1.update(gamepad1);

            // Actually drive the robot
            robot.drive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            //Speed down toggle
            if(gamepadEx1.a.isNewlyReleased() && robot.drive.getMaxSpeed() > 0){
                robot.intake.setMaxSpeed(robot.intake.getMaxSpeed() - 0.1);
            }

            // Speed up toggle
            if(gamepadEx1.b.isNewlyPressed() && robot.drive.getMaxSpeed() < 1){
                robot.intake.setMaxSpeed(robot.intake.getMaxSpeed() + 0.1);
            }

            if (gamepad1.right_bumper){
                robot.intake.intake();
            } else {
                robot.intake.stop();
            }
            if (gamepad1.dpad_up) {
                robot.intake.setHeight(robot.intake.UP);
            } else if (gamepad1.dpad_down) {
                robot.intake.setHeight(robot.intake.DOWN);
            }
            robot.arm.down();

            // Show the speed to the user
            telemetry.addData("Max Speed:", robot.intake.getMaxSpeed());
            telemetry.addLine("Press A to increase and B to decrease");
            telemetry.update();
        }

    }
}
