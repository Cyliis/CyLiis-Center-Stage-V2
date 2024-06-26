package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Intake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final ActiveIntake activeIntake;
    private final DropDown dropDown;
    private final Extendo extendo;
    private final BottomGripper bottomGripper;
    private final TopGripper topGripper;

    public static double delay = 0.1;

    public Intake(ActiveIntake activeIntake, DropDown dropDown, Extendo extendo, BottomGripper bottomGripper, TopGripper topGripper){
        this.activeIntake = activeIntake;
        this.dropDown = dropDown;
        this.extendo = extendo;
        this.bottomGripper = bottomGripper;
        this.topGripper = topGripper;
        state=State.IDLE;
    }

    @Override
    public void initUpdate() {
        if(!ENABLED) return;

         activeIntake.initUpdate();
         dropDown.initUpdate();
         extendo.initUpdate();
    }

    @Override
    public void atStart() {
        if(!ENABLED) return;

        activeIntake.atStart();
        dropDown.atStart();
        extendo.atStart();
    }

    public enum State {
        IDLE,
        START_INTAKE, PREPARE_DROPDOWN, OPENING_GRIPPERS, INTAKE, AUTO_REVERSE,
        STOP_INTAKE,REVERSE_DELAY ,REVERSE, GOING_IN, CLOSING_GRIPPERS
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;

        if(state == State.GOING_IN && newState == State.CLOSING_GRIPPERS) activeIntake.setState(ActiveIntake.State.PUSH);

        this.state = newState;

        switch (state){
            case IDLE:
                dropDown.setState(DropDown.State.UP);
                activeIntake.setState(ActiveIntake.State.IDLE);
                break;
            case START_INTAKE:
                setState(State.PREPARE_DROPDOWN);
                break;
            case PREPARE_DROPDOWN:
                dropDown.setState(DropDown.State.INTAKE);
                setState(State.OPENING_GRIPPERS);
                break;
            case OPENING_GRIPPERS:
                if(bottomGripper.getState() != BottomGripper.State.OPEN) bottomGripper.setState(BottomGripper.State.OPENING);
                if(topGripper.getState() != TopGripper.State.OPEN) topGripper.setState(TopGripper.State.OPENING);
                break;
            case INTAKE:
                activeIntake.setState(ActiveIntake.State.RUNNING);
                break;
            case STOP_INTAKE:
                dropDown.setState(DropDown.State.UP);
                activeIntake.setState(ActiveIntake.State.IDLE);
                setState(State.IDLE);
                break;
            case REVERSE_DELAY:
                dropDown.setState(DropDown.State.UP);
                break;
            case AUTO_REVERSE:
            case REVERSE:
                dropDown.setState(DropDown.State.UP);
                activeIntake.setState(ActiveIntake.State.REVERSE);
                break;
            case GOING_IN:
                extendo.setState(Extendo.State.GOING_IN);
                break;
            case CLOSING_GRIPPERS:
//                bottomGripper.setState(BottomGripper.State.CLOSING);
//                topGripper.setState(TopGripper.State.CLOSING);
                break;
        }
        timer.reset();
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
            case OPENING_GRIPPERS:
                if(bottomGripper.getState() == BottomGripper.State.OPEN && topGripper.getState() == TopGripper.State.OPEN)
                    setState(State.INTAKE);
                break;
            case GOING_IN:
                if(extendo.getState() == Extendo.State.IN) {
                    setState(State.CLOSING_GRIPPERS);
                }
                break;
            case IDLE:
                activeIntake.setState(ActiveIntake.State.IDLE);
                break;
            case CLOSING_GRIPPERS:
                if(activeIntake.getState()!= ActiveIntake.State.PUSH && bottomGripper.getState() == BottomGripper.State.OPEN && topGripper.getState() == TopGripper.State.OPEN){
                    topGripper.setState(TopGripper.State.CLOSING);
                    bottomGripper.setState(BottomGripper.State.CLOSING);
                }
                if(bottomGripper.getState() == BottomGripper.State.CLOSED && topGripper.getState() == TopGripper.State.CLOSED) {
                    if (activeIntake.getState() == ActiveIntake.State.IDLE) setState(State.IDLE);
                    if (activeIntake.getState() == ActiveIntake.State.REVERSE) setState(State.REVERSE);
                    if (activeIntake.getState() == ActiveIntake.State.RUNNING) setState(State.INTAKE);
                }
                break;
            case REVERSE_DELAY:
                if(timer.seconds() >= delay)
                    setState(State.REVERSE);
                break;
        }
    }

    @Override
    public void updateHardware() {
        activeIntake.update();
        dropDown.update();
        extendo.update();
    }

    @Override
    public void emergencyStop() {
        if(!ENABLED) return;

        activeIntake.emergencyStop();
        dropDown.emergencyStop();
        extendo.emergencyStop();
    }
}
