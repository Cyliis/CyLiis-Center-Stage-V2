package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

@Config
public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State{
        DOWN, GOING_UP_CLOSE, EXTEND_CLOSE1, EXTEND_CLOSE2, GOING_UP_FAR, EXTEND_FAR, LIFT_GOING_UP,
        UP, CHANGING_LIFT_POSITION, GOING_DOWN, HOME_TURRET, RETRACT_GO_PASSTHROUGH, GO_DOWN,
        GO_PURPLE, GOING_PASSTHROUGH, EXTENDING_CLOSE, LIFT_GOING_PURPLE_POSITION, PURPLE
    }

    State state;

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
                break;
            case UP:
                lift.setState(Lift.State.GOING_UP);
                turret.setState(Turret.State.BACKDROP);
                break;
            case GOING_DOWN:
                setState(State.HOME_TURRET);
                break;
            case HOME_TURRET:
                turret.setState(Turret.State.GOING_MIDDLE);
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
                extension.setState(Extension.State.GOING_CLOSE);
                break;
            case LIFT_GOING_PURPLE_POSITION:
                lift.setState(Lift.State.GOING_PURPLE);
                break;

        }
    }

    public State getState(){
        return state;
    }

    private final Lift lift;
    private final Extension extension;
    private final Turret turret;

    public Outtake(Lift lift, Extension extension, Turret turret, State initialState){
        this.lift = lift;
        this.extension = extension;
        this.turret = turret;
        this.state = initialState;
    }

    @Override
    public void atStart(){
        if(!ENABLED) return;

        lift.atStart();
        extension.atStart();
        turret.atStart();

    }

    @Override
    public void initUpdate(){
        if(!ENABLED) return;

        updateState();
        lift.initUpdate();
        extension.initUpdate();
        turret.initUpdate();
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
                if(lift.encoder.getCurrentPosition() >= Lift.passthroughPosition + Lift.groundPos)
                    setState(State.EXTEND_CLOSE1);
                break;
            case GOING_UP_FAR:
                if(lift.encoder.getCurrentPosition() >= Lift.passthroughPosition + Lift.groundPos)
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
            case HOME_TURRET:
                if(turret.getState() == Turret.State.MIDDLE)
                    setState(State.RETRACT_GO_PASSTHROUGH);
                break;
            case RETRACT_GO_PASSTHROUGH:
                if(extension.getState() == Extension.State.IN){
                    setState(State.GO_DOWN);
                    break;
                }
                if(extension.getState() != Extension.State.GOING_IN){
                    if(lift.encoder.getCurrentPosition() - Lift.groundPos >= Lift.passthroughPosition - Lift.positionThresh/2)
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
                if(extension.getState() == Extension.State.CLOSE)
                    setState(State.LIFT_GOING_PURPLE_POSITION);
                break;
            case LIFT_GOING_PURPLE_POSITION:
                if(lift.getState() == Lift.State.PURPLE)
                    setState(State.PURPLE);
                break;

        }
    }

    @Override
    public void updateHardware() {
        lift.update();
        extension.update();
        turret.update();
    }
}

