package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Wrappers.Analog30cm;

@TeleOp(group="zz")
public class ExtendoSensorTest extends OpMode {
    Analog30cm sensor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //p
        sensor = new Analog30cm(hardwareMap.get(AnalogInput.class, "extendo sensor"));
    }
    @Override
    public void loop() {

        sensor.update();
        telemetry.addData("Voltage", sensor.voltage);
        telemetry.addData("Distance", sensor.getDistance());
        telemetry.update();
    }
}
