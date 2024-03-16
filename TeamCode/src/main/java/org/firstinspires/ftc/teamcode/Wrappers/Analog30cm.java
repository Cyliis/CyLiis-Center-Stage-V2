package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class Analog30cm {

    AnalogInput input;

    public Analog30cm(AnalogInput analogInput){
        this.input = analogInput;
    }

    private double distance;
    public double voltage;

    public void update(){
        voltage = input.getVoltage();
        voltage = Math.max(voltage, 0.5);
        distance = 5.1 + 3.8/Math.tan((voltage - 0.4) * 0.8);
    }

    public double getDistance(){
        return distance;
    }
}
