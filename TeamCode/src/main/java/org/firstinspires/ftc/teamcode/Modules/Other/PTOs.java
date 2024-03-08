package org.firstinspires.ftc.teamcode.Modules.Other;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class PTOs implements IRobotModule, IStateBasedModule {
    public static boolean ENABLED = true;

    private final CoolServo leftServo, rightServo;
    public static boolean leftReversed = false, rightReversed = false;

    public static double leftDisengagedPosition=0.15, leftEngagedPosition=0.53;
    public static double rightDisengagedPosition=0.5, rightEngagedPosition=0.06;


    public enum State{
        DISENGAGED(leftDisengagedPosition, rightDisengagedPosition),
        ENGAGED(leftEngagedPosition, rightEngagedPosition);

        public double leftPos, rightPos;

        State(double leftPos, double rightPos){
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

    private void updateStateValues(){
        State.DISENGAGED.leftPos = leftDisengagedPosition;
        State.DISENGAGED.rightPos = rightDisengagedPosition;
        State.ENGAGED.leftPos = leftEngagedPosition;
        State.ENGAGED.rightPos = rightEngagedPosition;
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

    public PTOs(Hardware hardware, State initialState){
        if(!ENABLED) {
            leftServo = null;
            rightServo = null;
        }
        else {
            leftServo = new CoolServo(hardware.sch2, leftReversed, initialState.leftPos);
            rightServo = new CoolServo(hardware.sch5, rightReversed, initialState.rightPos);
            leftServo.forceUpdate();
            rightServo.forceUpdate();
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
        leftServo.setPosition(state.leftPos);
        rightServo.setPosition(state.rightPos);

        leftServo.update();
        rightServo.update();
    }
}