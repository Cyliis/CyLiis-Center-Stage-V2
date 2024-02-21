package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

    public static double inPosition1 = 0.8, outPosition1 = 0.21, inPosition2 = 0.29, outPosition2 = 0.85;

    public static double profileMaxVelocity = 40, profileAcceleration = 32, profileDeceleration = 24;

    public static double d0 = 57.811, l1 = 110.04, l2 = 110, h1 = 0.465, h2 = 18;
    public static double limit = 161.48907;
    public static double range = 2.288146111923855;
    public static double direction = -1;

    public static double sensorOffset = 53;
    public static double linearAdd = 13.0/300.0;
    public static double turretLength = 0;
    public static double offset = 0;

    private final Rev2mDistanceSensor sensor;
    private final Localizer localizer;
    private final Hardware.Color color;

    public static double filterParameter = 0.5;
    private final LowPassFilter filter = new LowPassFilter(filterParameter, 0);

    public double sensorReading;

    public Vector extensionVector = new Vector(0,0);

    public double servoDelta = 0;

    public double calculateBackdropExtension(){
        double distance = sensorReading;

        Vector targetVector = new Vector(distance * Math.cos(localizer.getHeading()), distance * Math.sin(localizer.getHeading()));
        Vector turretVector = new Vector(0, (color == Hardware.Color.Blue?1:-1) * turretLength);
        Vector offsetVector = new Vector(offset * Math.cos(localizer.getHeading()), offset * Math.sin(localizer.getHeading()));

        extensionVector = targetVector.plus(turretVector.scaledBy(-1)).plus(offsetVector.scaledBy(-1));

        return Math.min(limit, extensionVector.getMagnitude());
    }

    public double calculateAngle(double distance){
        double a = Math.sqrt((distance + d0) * (distance + d0) + (h2-h1) * (h2-h1));
        return Math.acos((a * a + l1 * l1 - l2 * l2)/(2 * a * l1));
    }

    public double getBackdropPosition(){
        double deltaFromIn = calculateAngle(calculateBackdropExtension()) - calculateAngle(0);
        servoDelta = (deltaFromIn * direction) / range;

        if(servoDelta < 0) servoDelta = 0;
        if(servoDelta > (outPosition2 - inPosition2)) servoDelta = outPosition2-inPosition2;
        if(Double.isNaN(servoDelta)) servoDelta = 0;

        return inPosition2 + servoDelta;
    }

    public enum State{
        IN(inPosition1, inPosition2), GOING_IN(inPosition1, inPosition2, IN),
        CLOSE(outPosition1, inPosition2), GOING_CLOSE(outPosition1, inPosition2, CLOSE),
        FAR(outPosition1, outPosition2), GOING_FAR(outPosition1, outPosition2, FAR),
        BACKDROP(outPosition1, inPosition2);

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
        sensorReading = filter.getValue(sensor.getDistance(DistanceUnit.MM));
        sensorReading = (1 + linearAdd)*(sensorReading-sensorOffset);

        State.IN.position1 = inPosition1;
        State.IN.position2 = inPosition2;
        State.GOING_IN.position1 = inPosition1;
        State.GOING_IN.position2 = inPosition2;
        State.CLOSE.position1 = outPosition1;
        State.CLOSE.position2 = inPosition2;
        State.GOING_CLOSE.position1 = outPosition1;
        State.GOING_CLOSE.position2 = inPosition2;
        State.FAR.position1 = outPosition1;
        State.FAR.position2 = outPosition2;
        State.GOING_FAR.position1 = outPosition1;
        State.GOING_FAR.position2 = outPosition2;
        State.BACKDROP.position1 = outPosition1;
        State.BACKDROP.position2 = getBackdropPosition();
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
        else servo1 = new CoolServo(hardware.seh0, reversedServo1, profileMaxVelocity, profileAcceleration, profileDeceleration, initialState.position1);
        if(!ENABLED) servo2 = null;
        else servo2 = new CoolServo(hardware.seh1, reversedServo2, profileMaxVelocity, profileAcceleration, profileDeceleration, initialState.position2);
        if(ENABLED){
            servo1.forceUpdate();
            servo2.forceUpdate();
        }
        if(ENABLED) sensor = hardware.outtakeSensor;
        else sensor = null;

        localizer = hardware.localizer;
        color = hardware.color;

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
        if(servo1.getTimeToMotionEnd() == 0 && servo2.getTimeToMotionEnd() == 0)
            setState(state.nextState);
    }

    @Override
    public void updateHardware() {
        if(state == State.BACKDROP) {
            servo2.cachedPosition = state.position2;
            servo2.forceUpdate();
        }

        servo1.setPosition(state.position1);
        servo2.setPosition(state.position2);

        servo1.update();
        servo2.update();
    }
}
