package org.firstinspires.ftc.teamcode.Modules.Other;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class Climb implements IRobotModule, IStateBasedModule {
    public static boolean ENABLED = true;

    private final CoolServo leftServo, rightServo, latchServo;
    public static boolean leftReversed = false, rightReversed = false, latchReversed = false;

    public static double leftDisengagedPosition=0.05, leftEngagedPosition=0.5;
    public static double rightDisengagedPosition=0.55, rightEngagedPosition=0.06;
    public static double latchOpenPosition = 1, latchClosedPosition = 1;

    public enum State{
        DISENGAGED(leftDisengagedPosition, rightDisengagedPosition, latchClosedPosition),
        HOOKS_DEPLOYED(leftDisengagedPosition, rightDisengagedPosition, latchOpenPosition),
        ENGAGED(leftEngagedPosition, rightEngagedPosition, latchOpenPosition);

        public double leftPos, rightPos, latchPos;

        State(double leftPos, double rightPos, double latchPos){
            this.leftPos = leftPos;
            this.rightPos = rightPos;
            this.latchPos = latchPos;
        }
    }

    private void updateStateValues(){
        State.DISENGAGED.leftPos = leftDisengagedPosition;
        State.DISENGAGED.rightPos = rightDisengagedPosition;
        State.DISENGAGED.latchPos = latchClosedPosition;
        State.HOOKS_DEPLOYED.leftPos = leftDisengagedPosition;
        State.HOOKS_DEPLOYED.rightPos = rightDisengagedPosition;
        State.HOOKS_DEPLOYED.latchPos = latchOpenPosition;
        State.ENGAGED.leftPos = leftEngagedPosition;
        State.ENGAGED.rightPos = rightEngagedPosition;
        State.ENGAGED.latchPos = latchOpenPosition;
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

    public Climb(Hardware hardware, State initialState){
        if(!ENABLED) {
            leftServo = null;
            rightServo = null;
            latchServo = null;
        }
        else {
            leftServo = new CoolServo(hardware.sch2, leftReversed, initialState.leftPos);
            rightServo = new CoolServo(hardware.sch5, rightReversed, initialState.rightPos);
            latchServo = new CoolServo(hardware.seh5, latchReversed, initialState.latchPos);
            leftServo.forceUpdate();
            rightServo.forceUpdate();
            latchServo.forceUpdate();
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
        latchServo.setPosition(state.latchPos);

        leftServo.update();
        rightServo.update();
        latchServo.update();
    }
}