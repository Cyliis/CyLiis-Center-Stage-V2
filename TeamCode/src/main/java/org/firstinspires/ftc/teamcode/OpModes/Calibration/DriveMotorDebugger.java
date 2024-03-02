package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class DriveMotorDebugger extends OpMode {
    DcMotorEx m0,m1,m2,m3;

    @Override
    public void init() {
        m0 = hardwareMap.get(DcMotorEx.class, "eh0");
        m1 = hardwareMap.get(DcMotorEx.class, "eh1");
        m2 = hardwareMap.get(DcMotorEx.class, "eh2");
        m3 = hardwareMap.get(DcMotorEx.class, "eh3");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) m0.setPower(1);
        if(gamepad1.dpad_left) m1.setPower(1);
        if(gamepad1.dpad_right) m2.setPower(1);
        if(gamepad1.dpad_down) m3.setPower(1);
    }
}
