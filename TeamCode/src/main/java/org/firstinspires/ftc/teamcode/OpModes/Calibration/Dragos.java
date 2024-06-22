package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Other.Plane;

@TeleOp(name="o guritza", group="zz")
@Config
public class Dragos extends LinearOpMode {

    public static double closePos=0.91 , openPos=0.84;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo;
        servo=hardwareMap.get(Servo.class , "seh1");
        servo.setPosition(closePos);
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.y || gamepad2.y)
            {
                if(servo.getPosition()==openPos)servo.setPosition(closePos);
                else servo.setPosition(openPos);
            }
        }
    }
}
