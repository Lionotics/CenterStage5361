package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;

// Class for cleaner code
public class Robot {
    public Arm arm = new Arm();
    public Airplane airplane = new Airplane();
    public Climb climb = new Climb();
    public Slides slides = new Slides();
    public Intake intake = new Intake();
    public Drivetrain drive = new Drivetrain();
    public boolean isAuto = false;

    public Robot(boolean isAuto){
        this.isAuto = isAuto;
    }

    public void init(HardwareMap hwMap){
        // Uncomment as hardware is setup and ready
        arm.init(hwMap);
//        airplane.init(hwMap);
        climb.init(hwMap);
        slides.init(hwMap);
        intake.init(hwMap);
        // If we are in auto, assume roadrunner is handling the drivetrain
        if(!isAuto) {
            drive.init(hwMap);
        }
    }
}
