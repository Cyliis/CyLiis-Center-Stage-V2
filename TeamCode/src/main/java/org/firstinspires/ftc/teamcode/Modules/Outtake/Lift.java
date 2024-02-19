package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Lift implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolMotor leftMotor, rightMotor;
    public static boolean leftMotorReversed = true, rightMotorReversed = false;
    public final Encoder encoder;
    public static boolean encoderReversed = false;

    public static int groundPos = 0, firstLevel = 250, increment = 70, level = 0, positionThresh = 10, passthroughPosition = 150;

    public static double resetPower = -0.5, velocityThreshold = 0;

    public static PIDCoefficients pid = new PIDCoefficients(0.015,0.15,0.0007);
    public static double ff1 = 0.00007, ff2 = 0.0002;

    public enum State{
        DOWN(0), RESETTING(0, DOWN), GOING_DOWN(0, RESETTING),
        UP(firstLevel + increment * level), GOING_UP(firstLevel + increment * level, UP),
        PASSTHROUGH(passthroughPosition), GOING_PASSTHROUGH(passthroughPosition, PASSTHROUGH);

        public int position;
        public final State nextState;

        State(int position){
            this.position = position;
            this.nextState = this;
        }

        State(int position, State nextState){
            this.position = position;
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
        this.state = newState;
    }

    private void updateStateValues(){
        State.UP.position = firstLevel + increment * level;
        State.GOING_UP.position = firstLevel + increment * level;
    }

    public Lift(Hardware hardware, State initialState){
        if(!ENABLED) leftMotor = null;
        else leftMotor = new CoolMotor(hardware.mch2, CoolMotor.RunMode.PID, leftMotorReversed);
        if(!ENABLED) rightMotor = null;
        else rightMotor = new CoolMotor(hardware.mch1, CoolMotor.RunMode.PID, rightMotorReversed);

        if(!ENABLED) encoder = null;
        else encoder = hardware.ech1;
        if(encoderReversed) encoder.setDirection(Encoder.Direction.REVERSE);

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
    public void updateState() {
        if(state == State.RESETTING){
            if(Math.abs(encoder.getRawVelocity()) <= velocityThreshold){
                groundPos = encoder.getCurrentPosition();
                updateStateValues();
                state = state.nextState;
            }
        }
        else if(Math.abs(state.nextState.nextState.position - encoder.getCurrentPosition()) <= positionThresh)
            state = state.nextState;
    }

    public int target = 0;

    @Override
    public void updateHardware() {

        if(state == State.RESETTING){
            leftMotor.setMode(CoolMotor.RunMode.RUN);
            rightMotor.setMode(CoolMotor.RunMode.RUN);
            leftMotor.setPower(resetPower);
            rightMotor.setPower(resetPower);
        }else{
            target = state.position + groundPos;

            leftMotor.setMode(CoolMotor.RunMode.PID);
            rightMotor.setMode(CoolMotor.RunMode.PID);
            leftMotor.setPIDF(pid, ff1 + ff2 * (double)(target - groundPos));
            rightMotor.setPIDF(pid, ff1 + ff2 * (double)(target - groundPos));
            leftMotor.calculatePower(encoder.getCurrentPosition(), target);
            rightMotor.calculatePower(encoder.getCurrentPosition(), target);
        }
        leftMotor.update();
        rightMotor.update();

    }
}
