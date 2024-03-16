package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Lift implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public final CoolMotor leftMotor, rightMotor;
    public static boolean leftMotorReversed = false, rightMotorReversed = true;

    public static double power = 1;
    public static int groundPosLeft = 0, groundPosRight = 0, firstLevel = 295, increment = 82, level = 0, positionThresh = 12,
            passthroughPosition = 250, purplePosition = 20;

    public static double resetPower = -0.5, maxHoldPower = -0.2, velocityThreshold = 0;

//    public static PIDCoefficients pid = new PIDCoefficients(0.02,0.1,0.0006);
//    public static double ff1 = 0.05, ff2 = 0.00003;

    public static PIDFCoefficients pidf = new PIDFCoefficients(25,0.1,0,0);

    private final ElapsedTime timer = new ElapsedTime();
    public static double timeOut = 0.15;

    public static double maxVelocity = 12000, acceleration = 10000, deceleration = 3500;
    public final AsymmetricMotionProfile profile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);
    public static boolean profiled = false;
    public static boolean defaultProfiledState = false;

    public enum State{
        DOWN(0), RESETTING(0, DOWN), GOING_DOWN(0, RESETTING),
        UP(firstLevel + increment * level), GOING_UP(firstLevel + increment * level, UP),
        PASSTHROUGH(passthroughPosition), GOING_PASSTHROUGH(passthroughPosition, PASSTHROUGH),
        PURPLE(purplePosition), GOING_PURPLE(purplePosition, PURPLE);

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
        profile.setMotion(profile.getPosition(), newState.position, profile.getSignedVelocity());
        if(newState != State.DOWN){
            leftMotor.motor.motor.setMotorEnable();
            rightMotor.motor.motor.setMotorEnable();
        }
        timer.reset();
        this.state = newState;
    }

    private void updateStateValues(){
        State.UP.position = firstLevel + increment * level;
        State.GOING_UP.position = firstLevel + increment * level;
        State.PASSTHROUGH.position = passthroughPosition;
        State.GOING_PASSTHROUGH.position = passthroughPosition;
        State.PURPLE.position = purplePosition;
        State.GOING_PURPLE.position = purplePosition;
    }

    public Lift(Hardware hardware, State initialState){
        if(!ENABLED) leftMotor = null;
        else {
            leftMotor = new CoolMotor(hardware.meh1, CoolMotor.RunMode.RTP, leftMotorReversed);
            leftMotor.setPower(power);
            leftMotor.setTarget(initialState.position + groundPosLeft);
//            leftMotor.motor.motor.setTargetPositionTolerance(20);
        }
        if(!ENABLED) rightMotor = null;
        else {
            rightMotor = new CoolMotor(hardware.meh2, CoolMotor.RunMode.RTP, rightMotorReversed);
            rightMotor.setPower(power);
            rightMotor.setTarget(initialState.position + groundPosRight);
//            rightMotor.motor.motor.setTargetPositionTolerance(20);
        }
        profiled = defaultProfiledState;

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
    public void updateState() {
        if(state == State.RESETTING){
            if(Math.abs(leftMotor.motor.motor.getVelocity()) <= velocityThreshold && timer.seconds() >= timeOut){
                groundPosLeft = leftMotor.getCurrentPosition();
                groundPosRight = rightMotor.getCurrentPosition();
                updateStateValues();
                setState(State.DOWN);
            }
        }
        else if(Math.abs((state.position + groundPosLeft) - leftMotor.getCurrentPosition()) <= positionThresh)
            setState(state.nextState);
        if(state == State.GOING_DOWN && timer.seconds() >= timeOut){
            if(Math.abs(leftMotor.motor.motor.getVelocity()) <= velocityThreshold){
                groundPosLeft = leftMotor.getCurrentPosition();
                groundPosRight = rightMotor.getCurrentPosition();
                setState(State.RESETTING);
            }
        }
    }

    public int target = 0;

    @Override
    public void updateHardware() {

        if(state == State.DOWN){
            if(leftMotor.motor.motor.isMotorEnabled()) leftMotor.motor.motor.setMotorDisable();
            if(rightMotor.motor.motor.isMotorEnabled()) rightMotor.motor.motor.setMotorDisable();
        }

        profile.update();
        pidf.algorithm = MotorControlAlgorithm.LegacyPID;

        if(state == State.RESETTING){
            if(leftMotor.getMode() != CoolMotor.RunMode.RUN) leftMotor.setMode(CoolMotor.RunMode.RUN);
            if(rightMotor.getMode() != CoolMotor.RunMode.RUN) rightMotor.setMode(CoolMotor.RunMode.RUN);
            leftMotor.setPower(resetPower);
            rightMotor.setPower(resetPower);
        }else {
            if(!profiled)target = state.position;
            else target = (int)profile.getPosition();
//            leftMotor.setPIDF(pid, ff1 + ff2 * (double) (target - groundPos));
//            rightMotor.setPIDF(pid, ff1 + ff2 * (double) (target - groundPos));
            leftMotor.setPIDF(pidf);
            rightMotor.setPIDF(pidf);
//            leftMotor.calculatePIDPower(encoder.getCurrentPosition(), target);
//            rightMotor.calculatePIDPower(encoder.getCurrentPosition(), target);
            leftMotor.setTarget(target + groundPosLeft);
            rightMotor.setTarget(target + groundPosRight);
            if(leftMotor.power != power) leftMotor.power = power;
            if(rightMotor.power != power) rightMotor.power = power;
            if(leftMotor.getMode() != CoolMotor.RunMode.RTP) leftMotor.setMode(CoolMotor.RunMode.RTP);
            if(rightMotor.getMode() != CoolMotor.RunMode.RTP) rightMotor.setMode(CoolMotor.RunMode.RTP);
//            if(state == State.DOWN){
//                leftMotor.power = Math.max(leftMotor.power, maxHoldPower);
//                rightMotor.power = Math.max(rightMotor.power, maxHoldPower);
//            }
        }
        leftMotor.update();
        rightMotor.update();

    }
}
