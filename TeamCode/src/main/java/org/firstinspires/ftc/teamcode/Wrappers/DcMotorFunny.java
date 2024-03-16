package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DcMotorFunny {

    public DcMotorEx motor;
    private double power;
    private int target;

    final Object lock = new Object();

    public DcMotorFunny(DcMotorEx motor){
        this.motor = motor;
    }

    public void setPowerAsync(double power){
        synchronized (lock) {
            this.power = power;
        }
    }

    public void setTargetPositionAsync(int target){
        this.target = target;
    }

    public void updatePowerAsync(){
        synchronized (lock) {
            if(motor.getPower() != power)
                motor.setPower(power);
        }
    }

    public void updateTargetPositionAsync(){
        synchronized (lock){
            if(motor.getTargetPosition() != target)
                motor.setTargetPosition(target);
        }
    }
}
