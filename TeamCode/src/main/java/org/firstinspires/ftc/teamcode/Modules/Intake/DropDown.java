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
    public static double init = 0.32;
    public static double pos0 = 0.35, pos1 = 0.39, pos2 = 0.42, pos3 = 0.445, pos4 = 0.47;
//    public static double upPos = 0.4;
    public static double inDiff = 0.01;

    public static double upOffset = 0.05;

    public static boolean in = true;

    public enum State {
        UP(upPosition), INTAKE(intakePosition), INIT(init);

        public double position;

        State(double position) {
            this.position = position;
        }
    }

    private void updateStateValues() {

        double[] poses = new double[]{pos0, pos1, pos2, pos3, pos4};
        intakePosition = poses[index] - (in?1:0) * inDiff;
        upPosition = poses[index] + upOffset;
        State.UP.position = upPosition;
        State.INTAKE.position = intakePosition;
        State.INIT.position = init;
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
        else servo = new CoolServo(hardware.seh4, reversedServo, initialState.position);
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
