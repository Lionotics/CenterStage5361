package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Airplane extends Mechanism{
    private double READY;
    private double SHOOT;
    Servo airplaneShooter;
    @Override
    public void init(HardwareMap hwMap) {
        airplaneShooter = hwMap.servo.get("airplaneShooterTemp");
        airplaneShooter.setPosition(READY);
    }
    public void shootAirplane() {
        airplaneShooter.setPosition(SHOOT);
    }
}
