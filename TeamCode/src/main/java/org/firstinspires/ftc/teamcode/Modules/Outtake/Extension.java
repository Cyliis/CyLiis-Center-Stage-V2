package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Math.LowPassFilter;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Extension implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public final CoolServo servo1, servo2;
    public static boolean reversedServo1 = false, reversedServo2 = true;

    public static double inPosition1 = 0.78, outPosition1 = 0.16,
            inPosition2 = 0.29, purplePosition = 0.3, closePosition = 0.5, outPosition2 = 0.83, pokePosition = 0.83;

    public static double profileMaxVelocity = 40, profileAcceleration = 32, profileDeceleration = 24;

    public static double boopTime = 0.2;

    public enum State{
        IN(inPosition1, inPosition2), GOING_IN(inPosition1, inPosition2, IN),
        CLOSE(outPosition1, closePosition), GOING_CLOSE(outPosition1, closePosition, CLOSE),
        MID(outPosition1, inPosition2), GOING_MID(outPosition1, inPosition2, MID),
        FAR(outPosition1, outPosition2), GOING_FAR(outPosition1, outPosition2, FAR),
        BOOP(outPosition1, inPosition2),
        PURPLE(outPosition1, purplePosition), GOING_PURPLE(outPosition1, purplePosition, PURPLE),
        POKE(inPosition1, pokePosition), GOING_POKE(inPosition1, pokePosition, POKE);

        public double position1, position2;
        public final State nextState;

        State(double position1, double position2){
            this.position1 = position1;
            this.position2 = position2;
            this.nextState = this;
        }

        State(double position1, double position2, State nextState){
            this.position1 = position1;
            this.position2 = position2;
            this.nextState = nextState;
        }
    }

    private void updateStateValues(){

        State.IN.position1 = inPosition1;
        State.IN.position2 = inPosition2;
        State.GOING_IN.position1 = inPosition1;
        State.GOING_IN.position2 = inPosition2;
        State.CLOSE.position1 = outPosition1;
        State.CLOSE.position2 = closePosition;
        State.GOING_CLOSE.position1 = outPosition1;
        State.GOING_CLOSE.position2 = closePosition;
        State.FAR.position1 = outPosition1;
        State.FAR.position2 = outPosition2;
        State.GOING_FAR.position1 = outPosition1;
        State.GOING_FAR.position2 = outPosition2;
        State.GOING_POKE.position1 = inPosition1;
        State.GOING_POKE.position2 = pokePosition;
        State.POKE.position1 = inPosition1;
        State.POKE.position2 = pokePosition;
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

    public Extension(Hardware hardware, State initialState){
        if(!ENABLED) servo1 = null;
        else servo1 = new CoolServo(hardware.sch1, reversedServo1, profileMaxVelocity, profileAcceleration, profileDeceleration, initialState.position1);
        if(!ENABLED) servo2 = null;
        else servo2 = new CoolServo(hardware.sch0, reversedServo2, profileMaxVelocity, profileAcceleration, profileDeceleration, initialState.position2);
        if(ENABLED){
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
        updateHardware();
        updateState();
    }

    @Override
    public void updateState() {
        if(timer.seconds() >= boopTime && state == State.BOOP) setState(State.GOING_CLOSE);
        if(servo1.getTimeToMotionEnd() == 0 && servo2.getTimeToMotionEnd() == 0)
            setState(state.nextState);
    }

    @Override
    public void updateHardware() {
        servo1.setPosition(state.position1);
        servo2.setPosition(state.position2);

        servo1.update();
        servo2.update();
    }
}
