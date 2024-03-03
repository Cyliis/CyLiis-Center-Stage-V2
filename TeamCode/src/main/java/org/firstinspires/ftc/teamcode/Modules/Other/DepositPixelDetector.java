package org.firstinspires.ftc.teamcode.Modules.Other;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Wrappers.CoolDigitalSensor;

@Config
public class DepositPixelDetector {
    public static double doublePixelTime = 0.15;

    private final CoolDigitalSensor bb0, bb1;

    private final ElapsedTime doublePixelTimer = new ElapsedTime();

    public DepositPixelDetector(RobotModules robotModules){
        this.bb0 = robotModules.beamBreak0;
        this.bb1 = robotModules.beamBreak1;
        doublePixelTimer.startTime();
    }

    private int pixels = 0;

    public int getPixels(){
        return pixels;
    }

    public void update(){
        boolean detection0 = !bb0.getState();
        boolean detection1 = !bb1.getState();

        if(detection0 && detection1){
            if(doublePixelTimer.seconds() >= doublePixelTime) pixels = 2;
        }
        else {
            doublePixelTimer.reset();
            if(detection0 || detection1) pixels = 1;
            else pixels = 0;
        }
    }
}
