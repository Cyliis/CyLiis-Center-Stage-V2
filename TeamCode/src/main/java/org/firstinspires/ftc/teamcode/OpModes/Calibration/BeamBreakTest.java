package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Wrappers.CoolDigitalSensor;

@TeleOp
public class BeamBreakTest extends OpMode {
    CoolDigitalSensor breakBeam0, breakBeam1;
    
    @Override
    public void init() {
        breakBeam0 = new CoolDigitalSensor(hardwareMap.get(DigitalChannel.class, "bb0"));
        breakBeam1 = new CoolDigitalSensor(hardwareMap.get(DigitalChannel.class, "bb1"));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("bb0",breakBeam0.getState());
        telemetry.addData("bb1",breakBeam1.getState());
        telemetry.update();
    }
}
