package org.firstinspires.ftc.teamcode.Modules.Other;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class Hooks implements IRobotModule, IStateBasedModule {
    public static boolean ENABLED = true;

    private final CoolServo servo;
    public static boolean latchReversed = false;

    public static double latchOpenPosition1 = 1, latchOpenPosition2 = 0, latchClosedPosition = 0.5;

    public enum State{
        IDLE(latchClosedPosition), HOOK1(latchOpenPosition1), HOOK2(latchOpenPosition2);

        public double pos;

        State(double pos){
            this.pos = pos;
        }
    }

    private void updateStateValues(){
        State.IDLE.pos = latchClosedPosition;
        State.HOOK1.pos = latchOpenPosition1;
        State.HOOK2.pos = latchOpenPosition2;
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

    public Hooks(Hardware hardware, State initialState){
        if(!ENABLED) {
            servo = null;
        }
        else {
            servo = new CoolServo(hardware.sch5, latchReversed, initialState.pos);
            servo.forceUpdate();
        }
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
    public void updateState() {
        
    }

    @Override
    public void updateHardware() {
        servo.setPosition(state.pos);

        servo.update();
    }
}