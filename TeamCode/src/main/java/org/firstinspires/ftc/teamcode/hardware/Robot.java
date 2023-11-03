package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

// Class for cleaner code
public class Robot {
    public Arm arm;
//    public Airplane airplane;
//    public Climb climb;
//    public Slides slides;

    public Drivetrain drivetrain;
    public boolean isAuto = false;

    public Robot(){


    }
    public void init(HardwareMap hwMap){
        //        TODO: Uncomment as hardware is setup and ready
        arm.init(hwMap);
//        airplane.init(hwMap);
//        climb.init(hwMap);
//        slides.init(hwMap);

        drivetrain.init(hwMap);
    }
}
