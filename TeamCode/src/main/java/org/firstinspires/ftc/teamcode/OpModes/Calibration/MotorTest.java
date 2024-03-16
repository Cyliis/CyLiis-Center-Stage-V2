package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class MotorTest extends OpMode {
    DcMotorEx motor;

    public static double power = 0.5;

    public static boolean chub = false;
    public static int port = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "eh0");
    }

    @Override
    public void loop() {
        motor = hardwareMap.get(DcMotorEx.class,(chub?"c":"e") + "h" + String.valueOf(port));
        motor.setPower(power);
    }
}
