package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.helpers.GamepadEx;

@TeleOp()
@Config
public class SlidesTesting extends LinearOpMode {
    Robot robot = new Robot(false);
    GamepadEx gamepadEx1 = new GamepadEx();
    @Override
    public void runOpMode() throws InterruptedException {

        // Dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        robot.slides.autoMoveTo(0);
        waitForStart();
        while (opModeIsActive()){
            robot.slides.pidLoop();

            telemetry.addData("Position", robot.slides.getPosition());
            telemetry.addData("Target",robot.slides.getTarget());
            telemetry.addData("state",robot.slides.getLiftState());
            telemetry.update();

            }
        }
}

