package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

@Config
public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State{
        DOWN, GOING_UP_CLOSE, EXTEND_CLOSE, GOING_UP_FAR, EXTEND_FAR,
        UP, CHANGING_LIFT_POSITION, GOING_DOWN, HOME_TURRET, RETRACT_GO_PASSTHROUGH, GO_DOWN
    }

    State state;

    public void setState(State newState){
        if(state == newState) return;

        state = newState;

        switch (newState){
            case GOING_UP_CLOSE:
            case GOING_UP_FAR:
            case CHANGING_LIFT_POSITION:
                lift.setState(Lift.State.GOING_UP);
                break;
            case EXTEND_CLOSE:
                extension.setState(Extension.State.GO_CLOSE);
                break;
            case EXTEND_FAR:
                extension.setState(Extension.State.GO_FAR);
                break;
            case UP:
                turret.setState(Turret.State.BACKDROP);
                break;
            case GOING_DOWN:
                setState(State.HOME_TURRET);
                break;
            case HOME_TURRET:
                turret.setState(Turret.State.GOING_MIDDLE);
                break;
            case RETRACT_GO_PASSTHROUGH:
                extension.setState(Extension.State.GOING_IN);
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                break;
            case GO_DOWN:
                lift.setState(Lift.State.GOING_DOWN);
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
        lift.atStart();
        extension.atStart();
        turret.atStart();

    }

    @Override
    public void initUpdate(){
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
                if(lift.encoder.getCurrentPosition() >= Lift.passthroughPosition)
                    setState(State.EXTEND_CLOSE);
                break;
            case GOING_UP_FAR:
                if(lift.encoder.getCurrentPosition() >= Lift.passthroughPosition)
                    setState(State.EXTEND_FAR);
                break;
            case CHANGING_LIFT_POSITION:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case EXTEND_CLOSE:
            case EXTEND_FAR:
                if(extension.getState() == extension.getState().nextState)
                    setState(State.UP);
                break;
            case GOING_DOWN:
                if(turret.getState() == Turret.State.MIDDLE)
                    setState(State.RETRACT_GO_PASSTHROUGH);
                break;
            case RETRACT_GO_PASSTHROUGH:
                if(extension.getState() == Extension.State.IN)
                    setState(State.GO_DOWN);
                break;
            case GO_DOWN:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.DOWN);
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

