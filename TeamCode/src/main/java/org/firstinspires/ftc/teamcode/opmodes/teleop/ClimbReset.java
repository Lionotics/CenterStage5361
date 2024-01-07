package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@Disabled
@TeleOp(name = "Reset Climb")
public class ClimbReset extends LinearOpMode {
   Robot robot = new Robot(false);

   public enum STATE {
       RAISING,
       LOWERING,
       DONE

    }
    private STATE state = STATE.RAISING;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive() & state != STATE.DONE){

            switch (state){
                // Bring the climb back up
                case RAISING:
                    robot.climb.climbUp();
                    if(robot.climb.getPosition() >= robot.climb.CLIMB_UP - 10){
                        robot.climb.stop();

                        state = STATE.LOWERING;
                    }
                    break;

                case LOWERING:
                    robot.climb.hookDown();
                    robot.climb.climbDown();
                    if(robot.climb.getPosition() < 10){
                        robot.climb.stop();
                        state = STATE.DONE;
                    }
                    break;
            }
        }


    }
}
