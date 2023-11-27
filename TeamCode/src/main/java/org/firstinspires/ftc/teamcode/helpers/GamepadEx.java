package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {

    public Button a = new Button(false);
    public Button b = new Button(false);
    public Button x = new Button(false);
    public Button y = new Button(false);
    public Button up = new Button(false);
    public Button down = new Button(false);
    public Button left = new Button(false);
    public Button right = new Button(false);

    public GamepadEx(){}

    public void update(Gamepad updatedGamepad){

        this.a.update(updatedGamepad.a);
        this.b.update(updatedGamepad.b);
        this.x.update(updatedGamepad.x);
        this.y.update(updatedGamepad.y);
        this.up.update(updatedGamepad.dpad_up);
        this.down.update(updatedGamepad.dpad_down);
        this.left.update(updatedGamepad.dpad_left);
        this.right.update(updatedGamepad.dpad_right);

    }
}
