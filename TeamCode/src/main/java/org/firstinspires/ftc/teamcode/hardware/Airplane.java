package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Airplane extends Mechanism{

    private boolean hasShot = false;

    private double READY = 0.4;
    private double SHOOT = 0.5;
    private Servo airplaneShooter;
    @Override
    public void init(HardwareMap hwMap) {
        airplaneShooter = hwMap.servo.get("airplaneShooterTemp");
        airplaneShooter.setPosition(READY);
    }

    public void shootAirplane() {

        airplaneShooter.setPosition(SHOOT);
        hasShot = true;
    }

    public boolean hasShot(){
        return hasShot;
    }
}
