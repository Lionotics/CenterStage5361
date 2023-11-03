package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp
public class DrivetrainTesting extends LinearOpMode
{
//    Robot robot = new Robot();
    Drivetrain drive = new Drivetrain();
    GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        waitForStart();
        drive.setMaxSpeed(1);

        while (opModeIsActive()){


            // Actually drive the robot
            drive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
//            gamepadEx1.update(gamepad1);
//
//
//            // Speed down toggle
//            if(gamepadEx1.a.isNewlyReleased() && drive.getMaxSpeed() > 0){
//                drive.setMaxSpeed(drive.getMaxSpeed() - 0.1);
//            }
//            // Speed up toggle
//            if(gamepadEx1.b.isNewlyPressed() && drive.getMaxSpeed() < 1){
//                drive.setMaxSpeed(drive.getMaxSpeed() + 0.1);
//            }
            // Show the speed to the user
            telemetry.addData("Max Speed:", drive.getMaxSpeed());
            telemetry.update();
            // update our gamepad
        }

    }
}