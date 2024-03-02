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

    public static double runningPower = 1, reversePower = -1;

    public static double pushTime = 0.1;

    public enum State{
        RUNNING(runningPower), IDLE(0), REVERSE(reversePower), PUSH(runningPower);

        public double power;

        State(double power){
            this.power = power;
        }
    }

    private void updateStateValues(){
        State.RUNNING.power = runningPower;
    }

    private State state;
    private State lastState;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
//        if(state!=State.PUSH) {
            this.lastState = state;
            this.state = newState;
//        }
//        else{
//            this.lastState = newState;
//        }
        timer.reset();
    }

    public ActiveIntake(Hardware hardware, State initialState){
        if(!ENABLED) motor = null;
        else motor = new CoolMotor(hardware.mch0, CoolMotor.RunMode.RUN, reversedMotor);
        timer.startTime();
        state = initialState;
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
    public void updateState() {
        if(state == State.PUSH && timer.seconds() >= pushTime) setState(lastState);
    }

    @Override
    public void updateHardware() {
        motor.setPower(state.power);

        motor.update();
    }
}
