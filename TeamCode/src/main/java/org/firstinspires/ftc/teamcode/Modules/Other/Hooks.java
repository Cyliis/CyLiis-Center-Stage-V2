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

    private final CoolServo servo1, servo2;

    public static double openPos1 = 0.75, openPos2 = 0.25, closedPos1 = 0.5, closedPos2 = 0.5;

    public enum State{
        CLOSED(closedPos1, closedPos2), OPEN(openPos1, openPos2);

        public double pos1, pos2;

        State(double pos1, double pos2){
            this.pos1 = pos1; this.pos2 = pos2;
        }
    }

    private void updateStateValues(){
        State.CLOSED.pos1 = closedPos1;
        State.CLOSED.pos2 = closedPos2;
        State.OPEN.pos1 = openPos1;
        State.OPEN.pos2 = openPos2;
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
            servo1 = null;
            servo2 = null;
        }
        else {
            servo1 = new CoolServo(hardware.seh3, false, initialState.pos1);
            servo2 = new CoolServo(hardware.seh5, false, initialState.pos1);
            servo1.forceUpdate();
            servo2.forceUpdate();
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
        servo1.setPosition(state.pos1);
        servo2.setPosition(state.pos2);

        servo1.update();
        servo2.update();
    }
}