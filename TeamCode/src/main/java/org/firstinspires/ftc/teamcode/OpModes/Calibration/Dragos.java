package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Other.Plane;

@TeleOp(name="o guritza", group="zz")
@Config
public class Dragos extends LinearOpMode {

    public static double closePos=0.505 , openPos=0.43;
    enum State{
        OPEN(openPos),
        CLOSED(closePos);
        double pos;
        State(double pos)
        {
            this.pos=pos;
        }

    }
    State state=State.CLOSED;
    @Override
    public void runOpMode() throws InterruptedException {

       boolean prev=false;
        Servo servo;
        servo=hardwareMap.get(Servo.class , "seh1");
        servo.setPosition(closePos);
        waitForStart();
        while(opModeIsActive())
        {
            if((gamepad1.y || gamepad1.x || gamepad1.a || gamepad1.b) && !prev)
            {
                switch(state)
                {
                    case OPEN:
                        state=State.CLOSED;
                        break;
                    case CLOSED:
                        state=State.OPEN;
                        break;
                }
                prev=true;
            }
            if(!(gamepad1.y || gamepad1.x || gamepad1.a || gamepad1.b)) prev=false;
            servo.setPosition(state.pos);
            state.OPEN.pos=openPos;
            state.CLOSED.pos=closePos;
            state=state;
        }

    }
}
