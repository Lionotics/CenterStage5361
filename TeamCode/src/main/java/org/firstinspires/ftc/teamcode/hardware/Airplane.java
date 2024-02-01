package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Airplane extends Mechanism{

    private boolean hasShot = false;

    public static double READY = 0;
    public static double SHOOT = 0.3;
    private Servo airplaneShooter;
    @Override
    public void init(HardwareMap hwMap) {
        airplaneShooter = hwMap.servo.get("airplane");
        airplaneShooter.setPosition(READY);
    }

    public void shootAirplane() {

        airplaneShooter.setPosition(SHOOT);
        hasShot = true;
    }

    public void reset(){
        airplaneShooter.setPosition(READY);
        hasShot = false;
    }


    public boolean hasShot(){
        return hasShot;
    }
}
