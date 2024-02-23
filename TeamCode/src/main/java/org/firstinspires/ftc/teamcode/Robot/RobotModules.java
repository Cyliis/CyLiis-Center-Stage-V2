package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Other.Climb;
import org.firstinspires.ftc.teamcode.Modules.Other.Plane;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;

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
    public final Climb climb;

    private final ArrayList<IRobotModule> modules = new ArrayList<>();

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
        climb = new Climb(hardware, Climb.State.DISENGAGED);

        modules.add(plane);
        modules.add(climb);

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
