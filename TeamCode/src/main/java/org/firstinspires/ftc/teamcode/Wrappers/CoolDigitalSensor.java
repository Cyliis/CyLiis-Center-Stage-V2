package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class CoolDigitalSensor {

    private DigitalChannel sensor;

    public CoolDigitalSensor(DigitalChannel sensor){
        this.sensor = sensor;
        this.sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState(){
        return sensor.getState();
    }
}
