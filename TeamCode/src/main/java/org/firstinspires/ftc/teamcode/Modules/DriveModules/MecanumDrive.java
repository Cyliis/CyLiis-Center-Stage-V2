package org.firstinspires.ftc.teamcode.Modules.DriveModules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Math.Angles;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;

@Config
public class MecanumDrive implements IRobotModule {

    public static boolean ENABLED = true;

    private final Localizer localizer;

    public final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public static boolean frontLeftMotorReversed = false, frontRightMotorReversed = true, backLeftMotorReversed = false, backRightMotorReversed = true;

    public static PIDCoefficients translationalPID = new PIDCoefficients(0.15,0 ,0),
            headingPID = new PIDCoefficients(2.5,0,0.19);
    public final PIDController tpid= new PIDController(0,0,0), hpid = new PIDController(0,0,0);

    public static double lateralMultiplier = 2;

    public double overallMultiplier = 1;

    public double velocityThreshold = 0.5;

    public static double ks = 0.03;

    private final VoltageSensor voltageSensor;
    public static double voltage = 0;

    public enum RunMode{
        PID, Vector, Climb
    }

    private RunMode runMode;

    public MecanumDrive(Hardware hardware, RunMode runMode, boolean brake){
        this.runMode = runMode;
        this.voltageSensor = hardware.voltageSensor;
        if(!ENABLED) {
            this.localizer = null;
            frontLeft = null;
            frontRight = null;
            backLeft = null;
            backRight = null;
            return;
        }

        this.localizer = hardware.localizer;
        frontLeft = new CoolMotor(hardware.meh3, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(hardware.meh0, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(hardware.meh2, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(hardware.meh1, CoolMotor.RunMode.RUN, backRightMotorReversed);

        if(brake){
            frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.runMode = runMode;
    }

    public Vector powerVector = new Vector();
    private Pose targetPose = new Pose();
    public Vector targetVector = new Vector();

    public void setTargetPose(Pose pose){
        this.targetPose = pose;
    }

    public void setTargetVector(Vector Vector){
        this.targetVector = Vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public Localizer getLocalizer(){
        return localizer;
    }

    public Pose getTargetPose(){
        return targetPose;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    public boolean reachedTarget(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return localizer.getPoseEstimate().getDistance(targetPose) <= tolerance;
    }

    public double diff;

    public boolean reachedHeading(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return Math.abs(Angles.normalize(targetPose.getHeading() - localizer.getHeading())) <= tolerance;
    }

    public boolean stopped(){
        return localizer.getVelocity().getMagnitude() <= velocityThreshold;
    }

    private void updatePowerVector(){
        switch (runMode){
            case Vector:
                powerVector = new Vector(targetVector.getX(), targetVector.getY(), targetVector.getZ());
                powerVector = Vector.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PID:
                Pose currentPose = localizer.getPredictedPoseEstimate();

                double xDiff = targetPose.getX() - currentPose.getX();
                double yDiff = targetPose.getY() - currentPose.getY();

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);

                double translationalPower = tpid.calculate(-distance, 0);

                powerVector = new Vector(translationalPower * Math.cos(Math.atan2(yDiff, xDiff)), translationalPower * Math.sin(Math.atan2(yDiff, xDiff)));
                powerVector = Vector.rotateBy(powerVector, currentPose.getHeading());

                double headingDiff = Angles.normalize(targetPose.getHeading() - currentPose.getHeading());

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);

                double headingPower = hpid.calculate(-headingDiff, 0);

                powerVector= new Vector(powerVector.getX(),powerVector.getY() * lateralMultiplier, headingPower);
                break;
            case Climb:
                powerVector = new Vector(targetVector.getX(), 0);
                break;
        }
        if(Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
            powerVector.scaleToMagnitude(1);
        powerVector.scaleBy(overallMultiplier);
    }

    private void updateMotors(){
        if(runMode != RunMode.Climb) {
            voltage = voltageSensor.getVoltage();
            double actualKs = ks * 12.0 / voltage;

            frontLeft.setPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));
            frontRight.setPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));
            backLeft.setPower((powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ()));
            backRight.setPower((powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
        } else {
            frontLeft.setPower(powerVector.getX());
            frontRight.setPower(powerVector.getX());
            backLeft.setPower(powerVector.getX());
            backRight.setPower(powerVector.getX());
        }
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updatePowerVector();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        powerVector = new Vector();
        updateMotors();
    }
}
