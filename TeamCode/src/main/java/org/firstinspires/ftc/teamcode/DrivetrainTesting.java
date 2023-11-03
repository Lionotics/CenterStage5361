package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
@TeleOp
public class DrivetrainTesting extends LinearOpMode {

    Drivetrain drive = new Drivetrain();
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            drive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
        }
    }
}
