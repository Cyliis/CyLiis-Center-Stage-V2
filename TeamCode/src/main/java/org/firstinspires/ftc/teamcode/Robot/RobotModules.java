package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Other.Hooks;
import org.firstinspires.ftc.teamcode.Modules.Other.PTOs;
import org.firstinspires.ftc.teamcode.Modules.Other.Plane;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Wrappers.CoolDigitalSensor;

import java.util.ArrayList;

public class RobotModules implements IRobotModule {

    public final MecanumDrive drive;

    public final ActiveIntake activeIntake;
    public final DropDown dropDown;
    public final Ramp ramp;
    public final Extendo extendo;
    public final BottomGripper bottomGripper;
    public final TopGripper topGripper;

    public final Intake intake;

    public final Extension extension;
    public final Lift lift;
    public final Turret turret;

    public final Outtake outtake;

    public final Plane plane;
    public final PTOs ptos;
    public final Hooks hooks;

    private final ArrayList<IRobotModule> modules = new ArrayList<>();

    public final CoolDigitalSensor beamBreak0, beamBreak1;

    public RobotModules(Hardware hardware, MecanumDrive drive){
        activeIntake = new ActiveIntake(hardware, ActiveIntake.State.IDLE);
        dropDown = new DropDown(hardware, DropDown.State.UP);
        ramp = new Ramp(hardware, Ramp.State.DOWN);
        extendo = new Extendo(hardware, Extendo.State.RESETTING);
        bottomGripper = new BottomGripper(hardware, BottomGripper.State.OPENING);
        topGripper = new TopGripper(hardware, TopGripper.State.OPENING);

        modules.add(bottomGripper);
        modules.add(topGripper);

        intake = new Intake(activeIntake, dropDown, ramp, extendo, bottomGripper, topGripper);

        modules.add(intake);

        extension = new Extension(hardware, Extension.State.IN);
        lift = new Lift(hardware, Lift.State.RESETTING);
        turret = new Turret(hardware, Turret.State.MIDDLE);

        outtake = new Outtake(lift, extension, turret, Outtake.State.DOWN);

        modules.add(outtake);

        plane = new Plane(hardware, Plane.State.CLOSED);
        ptos = new PTOs(hardware, PTOs.State.DISENGAGED);
        hooks = new Hooks(hardware, Hooks.State.IDLE);

        modules.add(plane);
        modules.add(ptos);
        modules.add(hooks);

        beamBreak0 = hardware.beamBreak0;
        beamBreak1 = hardware.beamBreak1;

        this.drive = drive;
    }

    public void telemetry(Telemetry telemetry){
        telemetry.addData("Lift level", Lift.level);
        telemetry.addData("Current position lift", lift.encoder.getCurrentPosition());
        telemetry.addData("Target position lift", lift.target);
        telemetry.addData("Outtake state", outtake.getState());
        telemetry.addData("Intake state", intake.getState());
        telemetry.addData("Lift state", lift.getState());
        telemetry.addData("Extendo state", extendo.getState());
        telemetry.addData("Active intake state", activeIntake.getState());
        telemetry.addData("Bottom gripper state", bottomGripper.getState());
        telemetry.addData("Top gripper state", topGripper.getState());
        telemetry.addData("PTOs state", ptos.getState());
        telemetry.addData("Hooks state", hooks.getState());
        telemetry.addData("Drive mode", drive.getRunMode());
        telemetry.addData("Lift power", lift.leftMotor.power);
        telemetry.addData("Voltage", Hardware.voltage);
        telemetry.addData("Left power lift", lift.leftMotor.power);
        telemetry.addData("Right power lift", lift.rightMotor.power);
        telemetry.addData("Power extendo", extendo.motor.power);
    }

    @Override
    public void initUpdate() {
        for(IRobotModule module: modules) module.initUpdate();
    }

    @Override
    public void atStart() {
        for(IRobotModule module: modules) module.atStart();
    }

    @Override
    public void update() {
        for(IRobotModule module: modules) module.update();
    }

    @Override
    public void emergencyStop() {
        for(IRobotModule module: modules) module.emergencyStop();
    }
}
