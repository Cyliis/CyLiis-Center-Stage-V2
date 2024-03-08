package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StickyGamepad {
    private final Gamepad gamepad;

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean left_bumper, right_bumper;
    public boolean left_stick_button, right_stick_button;
    public boolean back, ps;

    private boolean dpad_up_down, dpad_down_down, dpad_left_down, dpad_right_down;
    private boolean a_down, b_down, x_down, y_down;
    private boolean left_bumper_down, right_bumper_down;
    private boolean left_stick_button_down, right_stick_button_down;
    private boolean back_down, ps_down;

    public StickyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        dpad_up = gamepad.dpad_up && !dpad_up_down;
        dpad_up_down = gamepad.dpad_up;
        dpad_left = gamepad.dpad_left && !dpad_left_down;
        dpad_left_down = gamepad.dpad_left;
        dpad_down = gamepad.dpad_down && !dpad_down_down;
        dpad_down_down = gamepad.dpad_down;
        dpad_right = gamepad.dpad_right && !dpad_right_down;
        dpad_right_down = gamepad.dpad_right;
        a = gamepad.a && !a_down;
        a_down = gamepad.a;
        b = gamepad.b && !b_down;
        b_down = gamepad.b;
        x = gamepad.x && !x_down;
        x_down = gamepad.x;
        y = gamepad.y && !y_down;
        y_down = gamepad.y;
        left_bumper = gamepad.left_bumper && !left_bumper_down;
        left_bumper_down = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper && !right_bumper_down;
        right_bumper_down = gamepad.right_bumper;
        left_stick_button = gamepad.left_stick_button && !left_stick_button_down;
        left_stick_button_down = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button && !right_stick_button_down;
        right_stick_button_down = gamepad.right_stick_button;
        back = gamepad.back && !back_down;
        back_down = gamepad.back;
        ps = gamepad.ps && !ps_down;
        ps_down = gamepad.ps;
    }
}