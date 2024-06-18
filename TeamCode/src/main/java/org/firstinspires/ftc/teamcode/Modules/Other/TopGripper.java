package org.firstinspires.ftc.teamcode.Modules.Other;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class TopGripper implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolServo servo;
    public static boolean reversedServo = false;

    public static double openPosition = 0.93, closedPosition = 0.07;
    public static double motionTime = 0.15;

    public enum State{
        OPEN(openPosition), OPENING(openPosition, OPEN), CLOSED(openPosition), CLOSING(openPosition, CLOSED);

        public double position;
        public final State nextState;

        State(double position){
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState){
            this.position = position;
            this.nextState = nextState;
        }
    }

    private void updateStateValues(){
        State.OPEN.position = openPosition;
        State.OPENING.position = openPosition;
        State.CLOSED.position = closedPosition;
        State.CLOSING.position = closedPosition;
    }

    private State state;

    public final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
        this.state = newState;
        timer.reset();
    }

    public TopGripper(Hardware hardware, State initialState){
        if(!ENABLED) servo = null;
        else servo = new CoolServo(hardware.sch5, reversedServo, initialState.position);
        timer.startTime();
        setState(initialState);
        if(ENABLED) servo.forceUpdate();
    }

    @Override
    public void initUpdate() {
        update();
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
        if(timer.seconds() >= motionTime) setState(state.nextState);
    }

    @Override
    public void updateHardware() {
        servo.setPosition(state.position);

        servo.update();
    }
}
