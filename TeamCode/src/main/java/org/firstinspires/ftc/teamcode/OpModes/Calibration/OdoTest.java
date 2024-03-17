package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OdoTest extends OpMode {
    DcMotorEx odo1, odo2;

    @Override
    public void init() {
        odo1 = hardwareMap.get(DcMotorEx.class, "ch0");
        odo2 = hardwareMap.get(DcMotorEx.class, "ch3");
    }

    @Override
    public void loop() {
        telemetry.addData("1",odo1.getCurrentPosition());
        telemetry.addData("2",odo2.getCurrentPosition());
        telemetry.update();
    }
}
