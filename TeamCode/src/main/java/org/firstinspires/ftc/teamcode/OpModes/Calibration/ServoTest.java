package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@TeleOp
public class ServoTest extends OpMode {
    Servo servo;

    public static double position = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "sch3");
    }

    @Override
    public void loop() {
        servo.setPosition(position);
    }
}
