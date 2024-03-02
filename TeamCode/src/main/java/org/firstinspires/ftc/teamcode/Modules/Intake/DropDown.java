package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class DropDown implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolServo servo;
    public static boolean reversedServo = true;

    public static double intakePosition = 0.5, upPosition = 0.5;
    public static int index = 0;
    public static double pos0 = 0.65, pos1 = 0.54, pos2 = 0.44, pos3 = 0.35, pos4 = 0.24;
    public static double upPos0 = 0.26, upPos1 = 0.26, upPos2 = 0.21, upPos3 = 0.13, upPos4 = 0;


    public enum State {
        UP(upPosition), INTAKE(intakePosition);

        public double position;
        public final State nextState;

        State(double position) {
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState) {
            this.position = position;
            this.nextState = nextState;
        }
    }

    private void updateStateValues() {

        double[] poses = new double[]{pos0, pos1, pos2, pos3, pos4};
        intakePosition = poses[index];
        double[] upPoses = new double[]{upPos0, upPos1, upPos2, upPos3, upPos4};
        upPosition = upPoses[index];
        State.UP.position = upPosition;
        State.INTAKE.position = intakePosition;
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (newState == state) return;
        this.state = newState;
        timer.reset();
    }

    public DropDown(Hardware hardware, State initialState) {
        if (!ENABLED) servo = null;
            // maybe sch1
        else servo = new CoolServo(hardware.sch0, reversedServo, initialState.position);
        timer.startTime();
        setState(initialState);
        if (ENABLED) servo.forceUpdate();
    }

    @Override
    public void initUpdate() {
        update();
    }

    @Override
    public void update() {
        if (!ENABLED) return;

        updateStateValues();
        updateHardware();
        updateState();
    }

    @Override
    public void updateState() {

    }

    @Override
    public void updateHardware() {
        servo.setPosition(state.position);

        servo.update();
    }
}
