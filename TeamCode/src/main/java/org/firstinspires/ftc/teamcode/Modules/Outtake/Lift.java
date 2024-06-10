package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.data.DGrowArray;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Lift implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public final CoolMotor motor;
    public static boolean motorReversed = false;

    public static double power = 1;
    public static int groundPos = 0, firstLevel = 280, increment = 100, positionThresh = 25,
            passthroughPosition = 250, purplePosition = 0;

    public static double level = 0;

    public static double resetPower = -1, holdPower = -0.2, holdThresh = 3;

    private final ElapsedTime timer = new ElapsedTime();

    public enum State{
        DOWN(0), RESETTING(0, DOWN), GOING_DOWN(0, RESETTING),
        UP(firstLevel + increment * level), GOING_UP(firstLevel + increment * level, UP),
        PASSTHROUGH(passthroughPosition), GOING_PASSTHROUGH(passthroughPosition, PASSTHROUGH),
        PURPLE(purplePosition), GOING_PURPLE(purplePosition, PURPLE);

        public int position;
        public final State nextState;

        State(double position){
            this.position = (int)position;
            this.nextState = this;
        }

        State(double position, State nextState){
            this.position = (int)position;
            this.nextState = nextState;
        }
    }

    private State state;

    public State getState(){
        return state;
    }

    public void setState(State newState){
        updateStateValues();
        if(state == newState) return;
        timer.reset();
        this.state = newState;
    }

    private void updateStateValues(){
        State.UP.position = (int)(firstLevel + increment * level);
        State.GOING_UP.position = (int)(firstLevel + increment * level);
        State.PASSTHROUGH.position = passthroughPosition;
        State.GOING_PASSTHROUGH.position = passthroughPosition;
        State.PURPLE.position = purplePosition;
        State.GOING_PURPLE.position = purplePosition;
    }

    public Lift(Hardware hardware, State initialState){
        //port 2

        if(!ENABLED) motor = null;
        else {
            motor = new CoolMotor(hardware.meh2, CoolMotor.RunMode.RUN, motorReversed);
            motor.motor.motor.setCurrentAlert(8, CurrentUnit.AMPS);
            setState(initialState);
        }

        level = 0;

        timer.startTime();

        this.state = initialState;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateStateValues();
        updateState();
        updateHardware();
    }

    @Override
    public void initUpdate() {
        update();
    }

    @Override
    public void updateState() {
        if(state == State.GOING_DOWN){
            if(motor.motor.motor.isOverCurrent()){
                groundPos = motor.getCurrentPosition();
                setState(State.DOWN);
            }
        }
        else{
            if(Math.abs((motor.getCurrentPosition()- groundPos)-state.position) <= positionThresh)
                setState(state.nextState);
        }
    }

    public int target = 0;

    @Override
    public void updateHardware() {
        if(Hardware.voltage <= 9) motor.motor.motor.setCurrentAlert(7, CurrentUnit.AMPS);
        else motor.motor.motor.setCurrentAlert(8, CurrentUnit.AMPS);
        if(state == State.DOWN){
            if(motor.getMode() != CoolMotor.RunMode.RUN){
                motor.setMode(CoolMotor.RunMode.RUN);
            }
            if (motor.getCurrentPosition() > groundPos + holdThresh) motor.setPower(holdPower);
            else if(motor.getCurrentPosition() <= groundPos + holdThresh) motor.setPower(0);
        }
        else if(state == State.GOING_DOWN){
            if(motor.getMode() != CoolMotor.RunMode.RUN) motor.setMode(CoolMotor.RunMode.RUN);
            motor.setPower(resetPower);
        }else {
            target = state.position;
            motor.setTarget(target + groundPos);
            motor.setPower(power);
            if(motor.getMode() != CoolMotor.RunMode.RTP) motor.setMode(CoolMotor.RunMode.RTP);
        }
        motor.update();
    }
}
