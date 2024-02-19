package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.Angles;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class Turret implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = false;

    private final CoolServo servo;
    public static boolean reversedServo = false;

    public static double middle = 0.495;
    public static double range = Math.toRadians(255);
    public static double limit = Math.toRadians(45);

    public static double profileMaxVelocity = 20, profileAcceleration = 20;
    private final Localizer localizer;
    private final Hardware.Color color;

    public enum State{
        MIDDLE(middle), GOING_MIDDLE(middle, MIDDLE), BACKDROP(middle);

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
        double targetAngle = Angles.normalize(Math.PI/2.0 * (color == Hardware.Color.Blue ? 1 : -1));
        double currentAngle = Angles.normalize(localizer.getHeading());
        double deltaAngle = Math.min(Math.max(Angles.normalize(targetAngle - currentAngle), -limit), limit);
        State.BACKDROP.position = middle + deltaAngle * (1.0/range);
        State.MIDDLE.position = middle;
        State.GOING_MIDDLE.position = middle;
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

    public Turret(Hardware hardware, State initialState){
        if(!ENABLED) servo = null;
        else servo = new CoolServo(hardware.seh2, reversedServo, profileMaxVelocity, profileAcceleration, initialState.position);
        timer.startTime();
        setState(initialState);
        if(ENABLED) servo.forceUpdate();

        this.localizer = hardware.localizer;
        this.color = hardware.color;
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
        if(servo.getTimeToMotionEnd() == 0) state = state.nextState;
    }

    @Override
    public void updateHardware() {
        servo.setPosition(state.position);

        servo.update();
    }
}

