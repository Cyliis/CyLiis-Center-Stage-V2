package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DoubleStickyGamepad {
    private final Gamepad gamepad1, gamepad2;

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean left_bumper, right_bumper;
    public boolean left_stick_button, right_stick_button;
    public boolean back;

    private boolean dpad_up_down, dpad_down_down, dpad_left_down, dpad_right_down;
    private boolean a_down, b_down, x_down, y_down;
    private boolean left_bumper_down, right_bumper_down;
    private boolean left_stick_button_down, right_stick_button_down;
    private boolean back_down;

    public DoubleStickyGamepad(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update() {
        dpad_up = (gamepad1.dpad_up && gamepad2.dpad_up) && !dpad_up_down;
        dpad_up_down = (gamepad1.dpad_up && gamepad2.dpad_up);
        dpad_left = (gamepad1.dpad_left && gamepad2.dpad_left) && !dpad_left_down;
        dpad_left_down = (gamepad1.dpad_left && gamepad2.dpad_left);
        dpad_down = (gamepad1.dpad_down && gamepad2.dpad_down) && !dpad_down_down;
        dpad_down_down = (gamepad1.dpad_down && gamepad2.dpad_down);
        dpad_right = (gamepad1.dpad_right && gamepad2.dpad_right) && !dpad_right_down;
        dpad_right_down = (gamepad1.dpad_right && gamepad2.dpad_right);
        a = (gamepad1.a && gamepad2.a) && !a_down;
        a_down = (gamepad1.a && gamepad2.a);
        b = (gamepad1.b && gamepad2.b) && !b_down;
        b_down = (gamepad1.b && gamepad2.b);
        x = (gamepad1.x && gamepad2.x) && !x_down;
        x_down = (gamepad1.x && gamepad2.x);
        y = (gamepad1.y && gamepad2.y) && !y_down;
        y_down = (gamepad1.y && gamepad2.y);
        left_bumper = (gamepad1.left_bumper && gamepad2.left_bumper) && !left_bumper_down;
        left_bumper_down = (gamepad1.left_bumper && gamepad2.left_bumper);
        right_bumper = (gamepad1.right_bumper && gamepad2.right_bumper) && !right_bumper_down;
        right_bumper_down = (gamepad1.right_bumper && gamepad2.right_bumper);
        left_stick_button = (gamepad1.left_stick_button && gamepad2.left_stick_button) && !left_stick_button_down;
        left_stick_button_down = (gamepad1.left_stick_button && gamepad2.left_stick_button);
        right_stick_button = (gamepad1.right_stick_button && gamepad2.right_stick_button) && !right_stick_button_down;
        right_stick_button_down = (gamepad1.right_stick_button && gamepad2.right_stick_button);
        back = (gamepad1.back && gamepad2.back) && !back_down;
        back_down = (gamepad1.back && gamepad2.back);
    }
}
