package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Extendo implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public final CoolMotor motor1, motor2;
    public static boolean motor1Reversed = true, motor2Reversed = true;
    public final Encoder encoder;
    public static boolean encoderReversed = true;

    public static int zeroPos;
    public static int extendedPos;
    public static double extensionLimit = 1320;
    private double extensionPower = 0;

    public static double lockTimeOut = 3;

    public static double resetPower = -0.7,  positionThreshold = 30, inThreshold = 200;

    public static PIDFCoefficients PIDF = new PIDFCoefficients(0.006,0.08,0.0003,0);
    public static PIDFCoefficients aPIDF = new PIDFCoefficients(0.006,0.08,0.0003,0);

    private final ElapsedTime timer = new ElapsedTime();

    public static double timeOut = 0;

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
        State.LOCK.position = Math.max(0,extendedPos);
        State.GOING_LOCK.position = Math.max(0,extendedPos);
    }

    public Extendo(Hardware hardware, State initialState){
        //port 0,1, encoder in 0

        if(!ENABLED) {
            motor1 = null;
            motor2 = null;
        }
        else {
            motor1 = new CoolMotor(hardware.meh0, CoolMotor.RunMode.PID, motor1Reversed);
            motor1.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.motor.motor.setCurrentAlert(5, CurrentUnit.AMPS);
            motor2 = new CoolMotor(hardware.meh1, CoolMotor.RunMode.PID, motor2Reversed);
            motor2.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.motor.motor.setCurrentAlert(5, CurrentUnit.AMPS);
        }

        if(!ENABLED) encoder = null;
        else {
            encoder = hardware.eeh0;
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
        if(state == State.GOING_IN){
            if(motor1.motor.motor.isOverCurrent() && timer.seconds() >= timeOut){
                zeroPos = encoder.getCurrentPosition();
                timeOut = 0;
                setState(State.IN);
            }
        }
        else{
            if(state == State.GOING_LOCK && timer.seconds() >= lockTimeOut) setState(state.nextState);
            if(Math.abs((motor1.getCurrentPosition()- zeroPos)-state.position) <= positionThreshold)
                setState(state.nextState);
        }

        DropDown.in = Math.abs(encoder.getCurrentPosition() - zeroPos) <= inThreshold;
    }

    public int target = 0;

    @Override
    public void updateHardware() {
        if(Hardware.voltage <= 9) motor1.motor.motor.setCurrentAlert(4, CurrentUnit.AMPS);
        else motor1.motor.motor.setCurrentAlert(5, CurrentUnit.AMPS);
        if(Hardware.voltage <= 9) motor2.motor.motor.setCurrentAlert(4, CurrentUnit.AMPS);
        else motor2.motor.motor.setCurrentAlert(5, CurrentUnit.AMPS);

        if(state == State.GOING_IN){
            if(motor1.getMode() != CoolMotor.RunMode.RUN)motor1.setMode(CoolMotor.RunMode.RUN);
            if(motor2.getMode() != CoolMotor.RunMode.RUN)motor2.setMode(CoolMotor.RunMode.RUN);
            motor1.setPower(resetPower);
            motor2.setPower(resetPower);
        }
        else if(state == State.OUT){
            if(motor1.getMode() != CoolMotor.RunMode.RUN)motor1.setMode(CoolMotor.RunMode.RUN);
            if(motor2.getMode() != CoolMotor.RunMode.RUN)motor2.setMode(CoolMotor.RunMode.RUN);
            motor1.setPower(extensionPower);
            motor2.setPower(extensionPower);
        } else {
            target = state.position;
            int current = encoder.getCurrentPosition();
            if(motor1.getMode() != CoolMotor.RunMode.PID) motor1.setMode(CoolMotor.RunMode.PID);
            motor1.setPIDF(Hardware.AUTO?aPIDF:PIDF, Hardware.AUTO?aPIDF.f:PIDF.f * Math.signum(motor1.getPIDPower(current, target)));
            motor1.calculatePIDPower(current, target + zeroPos);
            if(motor2.getMode() != CoolMotor.RunMode.PID) motor2.setMode(CoolMotor.RunMode.PID);
            motor2.setPIDF(Hardware.AUTO?aPIDF:PIDF, Hardware.AUTO?aPIDF.f:PIDF.f * Math.signum(motor2.getPIDPower(current, target)));
            motor2.calculatePIDPower(current, target + zeroPos);
        }
        motor1.update();
        motor2.update();

    }
}