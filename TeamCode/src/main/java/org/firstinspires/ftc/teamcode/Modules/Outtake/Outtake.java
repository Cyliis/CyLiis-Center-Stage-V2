package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

@Config
public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public static double waitTime = 0.3;

    public enum State{
        DOWN, GOING_UP_CLOSE, EXTEND_CLOSE1, EXTEND_CLOSE2, GOING_UP_FAR, EXTEND_FAR, LIFT_GOING_UP,
        UP, CHANGING_LIFT_POSITION, GOING_DOWN, GOING_DOWN_SAFE, GOING_DOWN_DELAY, HOME_END_EFFECTOR, RETRACT_GO_PASSTHROUGH, GO_DOWN,
        GO_PURPLE, GOING_PASSTHROUGH, EXTENDING_CLOSE, LIFT_GOING_PURPLE_POSITION, PURPLE,
        GOING_POKE, GO_POKE_FROM_UP, EXTEND_POKE, LIFT_GOING_CORRECT_POSITION_POKE, POKE
    }

    State state;

    public ElapsedTime timer = new ElapsedTime();

    public void setState(State newState){
        if(state == newState) return;

        state = newState;

        switch (newState){
            case GOING_UP_CLOSE:
            case GOING_UP_FAR:
                if(Lift.State.GOING_UP.position < Lift.passthroughPosition) lift.setState(Lift.State.GOING_PASSTHROUGH);
                else lift.setState(Lift.State.GOING_UP);
                break;
            case CHANGING_LIFT_POSITION:
                lift.setState(Lift.State.GOING_UP);
                break;
            case EXTEND_CLOSE1:
                extension.setState(Extension.State.GOING_MID);
                break;
            case EXTEND_CLOSE2:
                extension.setState(Extension.State.GOING_CLOSE);
                break;
            case EXTEND_FAR:
                extension.setState(Extension.State.GOING_FAR);
                break;
            case LIFT_GOING_UP:
                lift.setState(Lift.State.GOING_UP);
                turret.setState(Turret.State.BACKDROP);
                pivot.setState(Pivot.State.ROTATED);
                break;
            case UP:
                lift.setState(Lift.State.GOING_UP);
                turret.setState(Turret.State.BACKDROP);
                break;
            case GOING_DOWN:
                setState(State.HOME_END_EFFECTOR);
                break;
            case HOME_END_EFFECTOR:
                turret.setState(Turret.State.GOING_MIDDLE);
                pivot.setState(Pivot.State.GOING_HOME);
                break;
            case RETRACT_GO_PASSTHROUGH:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                break;
            case GO_DOWN:
                lift.setState(Lift.State.GOING_DOWN);
                break;
            case GO_PURPLE:
                setState(State.GOING_PASSTHROUGH);
                break;
            case GOING_PASSTHROUGH:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                break;
            case EXTENDING_CLOSE:
                extension.setState(Extension.State.GOING_PURPLE);
                break;
            case LIFT_GOING_PURPLE_POSITION:
                lift.setState(Lift.State.GOING_PURPLE);
                break;
            case GOING_POKE:
                if(Lift.State.GOING_UP.position < Lift.passthroughPosition) lift.setState(Lift.State.GOING_PASSTHROUGH);
                else lift.setState(Lift.State.GOING_UP);
                break;
            case EXTEND_POKE:
                extension.setState(Extension.State.GOING_POKE);
                break;
            case LIFT_GOING_CORRECT_POSITION_POKE:
                lift.setState(Lift.State.GOING_UP);
                break;
            case GO_POKE_FROM_UP:
                turret.setState(Turret.State.GOING_MIDDLE);
                pivot.setState(Pivot.State.GOING_HOME);
                break;

        }

        timer.reset();
    }


    public State getState(){
        return state;
    }

    private final Lift lift;
    private final Extension extension;
    private final Turret turret;
    private final Pivot pivot;

    public Outtake(Lift lift, Extension extension, Turret turret, Pivot pivot, State initialState){
        this.lift = lift;
        this.extension = extension;
        this.turret = turret;
        this.pivot = pivot;
        this.state = initialState;
        timer.startTime();
    }

    @Override
    public void atStart(){
        if(!ENABLED) return;

        lift.atStart();
        extension.atStart();
        turret.atStart();
        pivot.atStart();

    }

    @Override
    public void initUpdate(){
        if(!ENABLED) return;

        updateState();
        lift.initUpdate();
        extension.initUpdate();
        turret.initUpdate();
        pivot.initUpdate();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        switch (state){
            case GOING_UP_CLOSE:
                if(lift.motor.getCurrentPosition() - Lift.passthroughPosition - Lift.groundPos >= -Lift.positionThresh)
                    setState(State.EXTEND_CLOSE1);
                break;
            case GOING_UP_FAR:
                if(lift.motor.getCurrentPosition() - Lift.passthroughPosition - Lift.groundPos >= -Lift.positionThresh)
                    setState(State.EXTEND_FAR);
                break;
            case CHANGING_LIFT_POSITION:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case EXTEND_CLOSE1:
                if(extension.getState() == Extension.State.MID)
                    setState(State.EXTEND_CLOSE2);
                break;
            case EXTEND_CLOSE2:
                if(extension.getState() == Extension.State.CLOSE)
                    setState(State.LIFT_GOING_UP);
                break;
            case EXTEND_FAR:
                if(extension.getState() == extension.getState().nextState)
                    setState(State.LIFT_GOING_UP);
                break;
            case LIFT_GOING_UP:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case HOME_END_EFFECTOR:
                if(turret.getState() == Turret.State.MIDDLE && pivot.getState() == Pivot.State.HOME)
                    setState(State.RETRACT_GO_PASSTHROUGH);
                break;
            case RETRACT_GO_PASSTHROUGH:
                if(extension.getState() == Extension.State.IN){
                    setState(State.GO_DOWN);
                    break;
                }
                if(extension.getState() != Extension.State.GOING_IN){
                    if(lift.motor.getCurrentPosition() - Lift.groundPos >= Lift.passthroughPosition - Lift.positionThresh/2)
                        extension.setState(Extension.State.GOING_IN);
                }
                break;
            case GO_DOWN:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.DOWN);
                break;
            case GOING_PASSTHROUGH:
                if(lift.getState() == Lift.State.PASSTHROUGH)
                    setState(State.EXTENDING_CLOSE);
                break;
            case EXTENDING_CLOSE:
                if(extension.getState() == Extension.State.PURPLE)
                    setState(State.LIFT_GOING_PURPLE_POSITION);
                break;
            case LIFT_GOING_PURPLE_POSITION:
                if(lift.getState() == Lift.State.PURPLE)
                    setState(State.PURPLE);
                break;
            case GOING_DOWN_DELAY:
                if(timer.seconds()>=waitTime)
                    setState(State.GOING_DOWN);
                break;
            case GOING_DOWN_SAFE:
                if(timer.seconds()>=waitTime && turret.getState() == Turret.State.BACKDROP)
                    turret.setState(Turret.State.GOING_MIDDLE);
                if(timer.seconds()>=waitTime && pivot.getState() == Pivot.State.ROTATED)
                    pivot.setState(Pivot.State.GOING_HOME);
                if(turret.getState() == Turret.State.MIDDLE && pivot.getState() == Pivot.State.HOME &&
                extension.getState() != Extension.State.IN && extension.getState() != Extension.State.GOING_IN)
                    extension.setState(Extension.State.GOING_IN);
                if(extension.getState() == Extension.State.IN)
                    setState(State.GOING_DOWN);
                break;
            case GOING_POKE:
                if(lift.motor.getCurrentPosition() - Lift.passthroughPosition - Lift.groundPos >= -Lift.positionThresh)
                    setState(State.EXTEND_POKE);
                break;
            case GO_POKE_FROM_UP:
                if(turret.getState() == Turret.State.MIDDLE && pivot.getState() == Pivot.State.HOME)
                    setState(State.EXTEND_POKE);
                break;
            case EXTEND_POKE:
                if(extension.getState() == Extension.State.POKE)
                    setState(State.POKE);
                break;
        }
    }

    @Override
    public void updateHardware() {
        lift.update();
        extension.update();
        turret.update();
        pivot.update();
    }
}

