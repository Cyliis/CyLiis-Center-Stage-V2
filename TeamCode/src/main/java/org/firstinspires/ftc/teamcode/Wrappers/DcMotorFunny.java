package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DcMotorFunny {

    public DcMotorEx motor;
    private double power;

    final Object lock = new Object();

    public DcMotorFunny(DcMotorEx motor){
        this.motor = motor;
    }

    public void setPowerAsync(double power){
        synchronized (lock) {
            this.power = power;
        }
    }

    public void updatePowerAsync(){
        synchronized (lock) {
            motor.setPower(power);
        }
    }
}
