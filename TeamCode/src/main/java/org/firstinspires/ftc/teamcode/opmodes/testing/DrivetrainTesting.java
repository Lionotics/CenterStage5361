package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp(group = "testers")
public class DrivetrainTesting extends LinearOpMode
{
    //    Robot robot = new Robot();
//    Drivetrain drive = new Drivetrain();
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.drive.setMaxSpeed(0.2);

        while (opModeIsActive()){

            // update our gamepad
            gamepadEx1.update(gamepad1);

            // Actually drive the robot
            robot.drive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

//             Speed down toggle
            if(gamepadEx1.a.isNewlyReleased() && robot.drive.getMaxSpeed() > 0){
                robot.drive.setMaxSpeed(robot.drive.getMaxSpeed() - 0.01);
            }
            // Speed up toggle
            if(gamepadEx1.b.isNewlyPressed() && robot.drive.getMaxSpeed() < 1){
                robot.drive.setMaxSpeed(robot.drive.getMaxSpeed() + 0.01);
            }

            // Show the speed to the user
            telemetry.addData("Max Speed:", robot.drive.getMaxSpeed());
            telemetry.addLine("Press A to increase and B to decrease");
            telemetry.update();
        }

    }
}