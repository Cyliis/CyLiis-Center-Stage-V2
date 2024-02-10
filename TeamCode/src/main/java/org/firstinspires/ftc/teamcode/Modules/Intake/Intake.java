package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Intake implements IStateBasedModule, IRobotModule {

    private final ActiveIntake activeIntake;
    private final DropDown dropDown;
    private final Ramp ramp;
    private final Extendo extendo;
    private final BottomGripper bottomGripper;
    private final TopGripper topGripper;

    public Intake(ActiveIntake activeIntake, DropDown dropDown, Ramp ramp, Extendo extendo, BottomGripper bottomGripper, TopGripper topGripper){
        this.activeIntake = activeIntake;
        this.dropDown = dropDown;
        this.ramp = ramp;
        this.extendo = extendo;
        this.bottomGripper = bottomGripper;
        this.topGripper = topGripper;
    }

    @Override
    public void initUpdate() {
         activeIntake.initUpdate();
         dropDown.initUpdate();
         ramp.initUpdate();
         extendo.initUpdate();
    }

    @Override
    public void atStart() {
        activeIntake.atStart();
        dropDown.atStart();
        ramp.atStart();
        extendo.atStart();
    }

    public enum State {
        IDLE,
        START_INTAKE, PREPARE_RAMP, OPENING_GRIPPERS, INTAKE,
        STOP_INTAKE, REVERSE, GOING_IN, CLOSING_GRIPPERS
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
        switch (state){
            case IDLE:
                dropDown.setState(DropDown.State.UP);
                activeIntake.setState(ActiveIntake.State.IDLE);
                break;
            case START_INTAKE:
                setState(State.PREPARE_RAMP);
                break;
            case PREPARE_RAMP:
                ramp.setState(Ramp.State.INTAKE);
                dropDown.setState(DropDown.State.INTAKE);
                setState(State.OPENING_GRIPPERS);
                break;
            case OPENING_GRIPPERS:
                bottomGripper.setState(BottomGripper.State.OPENING);
                topGripper.setState(TopGripper.State.OPENING);
                break;
            case INTAKE:
                activeIntake.setState(ActiveIntake.State.RUNNING);
                break;
            case STOP_INTAKE:
                dropDown.setState(DropDown.State.UP);
                activeIntake.setState(ActiveIntake.State.IDLE);
                setState(State.IDLE);
                break;
            case REVERSE:
                dropDown.setState(DropDown.State.UP);
                activeIntake.setState(ActiveIntake.State.REVERSE);
                break;
            case GOING_IN:
                extendo.setState(Extendo.State.GOING_IN);
                activeIntake.setState(ActiveIntake.State.REVERSE);
                break;
            case CLOSING_GRIPPERS:
                bottomGripper.setState(BottomGripper.State.CLOSING);
                topGripper.setState(TopGripper.State.CLOSING);
                break;
        }
        this.state = newState;
        timer.reset();
    }

    @Override
    public void update() {
        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        switch (state){
            case OPENING_GRIPPERS:
                if(bottomGripper.getState() == BottomGripper.State.OPEN && topGripper.getState() == TopGripper.State.OPEN)
                    setState(State.INTAKE);
                break;
            case GOING_IN:
                if(extendo.getState() == Extendo.State.IN) {
                    activeIntake.setState(ActiveIntake.State.IDLE);
                    setState(State.CLOSING_GRIPPERS);
                }
                break;
            case CLOSING_GRIPPERS:
                if(bottomGripper.getState() == BottomGripper.State.CLOSED && topGripper.getState() == TopGripper.State.CLOSED) {
                    if (activeIntake.getState() == ActiveIntake.State.IDLE) setState(State.IDLE);
                    if (activeIntake.getState() == ActiveIntake.State.REVERSE) setState(State.REVERSE);
                }
        }
    }

    @Override
    public void updateHardware() {
        activeIntake.update();
        dropDown.update();
        ramp.update();
        extendo.update();
    }

    @Override
    public void emergencyStop() {
        activeIntake.emergencyStop();
        dropDown.emergencyStop();
        ramp.emergencyStop();
        extendo.emergencyStop();
    }
}
