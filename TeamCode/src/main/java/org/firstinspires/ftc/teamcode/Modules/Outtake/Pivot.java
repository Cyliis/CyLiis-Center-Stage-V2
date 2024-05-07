package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.Angles;
import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class Pivot implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolServo servo;
    public static boolean reversedServo = false;

    public static double profileMaxVelocity = 20, profileAcceleration = 32;
    private final AsymmetricMotionProfile profile = new AsymmetricMotionProfile(profileMaxVelocity, profileAcceleration, profileAcceleration);
    public static double homePosition = 0.5;
    public static double rotatedLeft = 0.5, rotatedRight = 0.5, flipped = 0.5;
    public static int index = 1;

    private double[] poses = {rotatedLeft, homePosition, rotatedRight, flipped};

    public enum State{
        HOME(homePosition), GOING_HOME(homePosition, HOME), ROTATED(homePosition);

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
        State.HOME.position = homePosition;
        State.GOING_HOME.position = homePosition;
        State.ROTATED.position = poses[index];
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
        if(newState == State.GOING_HOME) profile.setMotion(state.position, homePosition, 0);
        this.state = newState;
        timer.reset();
    }

    public Pivot(Hardware hardware, State initialState){
        if(!ENABLED) servo = null;
        else servo = new CoolServo(hardware.sch4, reversedServo, initialState.position);
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
        updateHardware();
        updateState();
    }

    @Override
    public void updateState() {
        if(profile.getTimeToMotionEnd() == 0 && state == State.GOING_HOME) state = state.nextState;
    }

    @Override
    public void updateHardware() {
        profile.update();
        if(state != State.GOING_HOME)servo.setPosition(state.position);
        else servo.setPosition(profile.getPosition());

        servo.update();
    }
}

