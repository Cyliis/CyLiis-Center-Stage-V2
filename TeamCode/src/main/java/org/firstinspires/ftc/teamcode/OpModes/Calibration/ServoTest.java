package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group="zz")
public class ServoTest extends OpMode {
    Servo servo;

    public static double position = 0.5;

    public static boolean chub = false;
    public static int port = 0;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "seh5");
    }

    @Override
    public void loop() {
        servo = hardwareMap.get(Servo.class, "s" + (chub?"c":"e") + "h" + String.valueOf(port));
        servo.setPosition(position);
    }
}
