package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;

@Config
public class ActiveIntake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolMotor motor;
    public static boolean reversedMotor = false;

    public static double runningPower = 1, reversePower = -0.7;

    public enum State{
        RUNNING(runningPower), IDLE(0), REVERSE(reversePower);

        public double power;

        State(double power){
            this.power = power;
        }
    }

    private void updateStateValues(){
        State.RUNNING.power = runningPower;
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
        this.state = newState;
        timer.reset();
    }

    public ActiveIntake(Hardware hardware, State initialState){
        if(!ENABLED) motor = null;
        else motor = new CoolMotor(hardware.mch3, CoolMotor.RunMode.RUN, reversedMotor);
        timer.startTime();
        setState(initialState);
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateStateValues();
        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {}

    @Override
    public void updateHardware() {
        motor.setPower(state.power);

        motor.update();
    }
}
