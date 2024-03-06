package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Extendo implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public final CoolMotor motor;
    public static boolean motorReversed = true;
    public final Encoder encoder;
    public static boolean encoderReversed = true;

    public static int zeroPos;
    public static int extendedPos;
    public static double extensionLimit = 1320;
    private double extensionPower = 0;

    public static double resetPower = -1, velocityThreshold = 0, positionThreshold = 10, inThreshold = 100;

    public static PIDFCoefficients PIDF = new PIDFCoefficients(0.04,0.15,0.0007,0);
//    public static PIDFCoefficients inPIDF = new PIDFCoefficients(0.5,0,0,0.03);

    public static double timeOut = 0.1;
    private final ElapsedTime timer = new ElapsedTime();

    public enum State{
        IN(0), RESETTING(0, IN), GOING_IN(0, RESETTING),
        OUT(0), GOING_OUT(0, OUT),
        LOCK(extendedPos), GOING_LOCK(extendedPos, LOCK);

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
        timer.reset();
        this.state = newState;
    }

    private void updateStateValues(){
        State.LOCK.position = extendedPos;
        State.GOING_LOCK.position = extendedPos;
    }

    public Extendo(Hardware hardware, State initialState){
        if(!ENABLED) motor = null;
        else {
            motor = new CoolMotor(hardware.mch3, CoolMotor.RunMode.PID, motorReversed);
            motor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(!ENABLED) encoder = null;
        else {
            encoder = hardware.ech1;
            if(encoderReversed) encoder.setDirection(Encoder.Direction.REVERSE);
        }

        timer.startTime();
        this.state = initialState;
    }

    public void setPower(double power){
        this.extensionPower = power;
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
            if(Math.abs(encoder.getRawVelocity()) <= velocityThreshold && timer.seconds() >= timeOut){
                zeroPos = encoder.getCurrentPosition();
                updateStateValues();
                state = state.nextState;
            }
        }
        else if(state == State.GOING_IN && timer.seconds() >= timeOut){
            if(Math.abs(encoder.getRawVelocity()) <= velocityThreshold) setState(State.RESETTING);
        }
        else if(Math.abs((state.position + zeroPos) - encoder.getCurrentPosition()) <= positionThreshold)
            state = state.nextState;
    }

    public int target = 0;

    @Override
    public void updateHardware() {
        if(state == State.RESETTING){
            motor.setMode(CoolMotor.RunMode.RUN);
            motor.setPower(resetPower);
        }
        else if(state == State.OUT){
            motor.setMode(CoolMotor.RunMode.RUN);
            motor.setPower(extensionPower);
        }else {
            target = state.position + zeroPos;
            motor.setMode(CoolMotor.RunMode.PID);
            motor.setPIDF(PIDF, PIDF.f * Math.signum(motor.getPower(encoder.getCurrentPosition(), target)));
            motor.calculatePower(encoder.getCurrentPosition(), target);
        }
        motor.update();

    }
}
