package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Other.Plane;

@TeleOp(name="o guritza")
public class Dragos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo;
        servo=hardwareMap.get(Servo.class , "seh3");
        servo.setPosition(0.85);
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.y || gamepad2.y)
            {
                if(servo.getPosition()==0.75)servo.setPosition(0.85);
                else servo.setPosition(0.75);
            }
        }
    }
}
