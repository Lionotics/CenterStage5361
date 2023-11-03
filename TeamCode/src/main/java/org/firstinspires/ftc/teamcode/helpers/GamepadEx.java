package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {

    private Gamepad gamepad;

    public Button b,x,y,up,down,left,right = new Button();
    public Button a = new Button(false);

    public GamepadEx(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public void update(Gamepad gamepad){

        this.a.update(gamepad.a);
        this.b.update(gamepad.b);
        this.x.update(gamepad.x);
        this.y.update(gamepad.y);
        this.up.update(gamepad.dpad_up);
        this.down.update(gamepad.dpad_down);
        this.left.update(gamepad.dpad_left);
        this.right.update(gamepad.dpad_right);

    }
}
