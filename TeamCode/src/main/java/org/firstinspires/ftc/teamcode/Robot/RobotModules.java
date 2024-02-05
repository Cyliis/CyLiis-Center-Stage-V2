package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.TopGripper;

public class RobotModules implements IRobotModule {

    public final ActiveIntake activeIntake;
    public final DropDown dropDown;
    public final Ramp ramp;
    public final Extendo extendo;
    public final BottomGripper bottomGripper;
    public final TopGripper topGripper;

    public final Intake intake;

    public RobotModules(Hardware hardware){
        activeIntake = new ActiveIntake(hardware, ActiveIntake.State.IDLE);
        dropDown = new DropDown(hardware, DropDown.State.UP);
        ramp = new Ramp(hardware, Ramp.State.INTAKE);
        extendo = new Extendo(hardware, Extendo.State.GOING_IN);
        bottomGripper = new BottomGripper(hardware, BottomGripper.State.OPENING);
        topGripper = new TopGripper(hardware, TopGripper.State.OPENING);

        intake = new Intake(activeIntake, dropDown, ramp, extendo, bottomGripper, topGripper);
    }

    public void telemetry(Telemetry telemetry){

    }

    @Override
    public void initUpdate() {
        intake.initUpdate();
        bottomGripper.initUpdate();
        topGripper.initUpdate();
    }

    @Override
    public void atStart() {
        intake.atStart();
        bottomGripper.atStart();
        topGripper.atStart();
    }

    @Override
    public void update() {
        intake.update();
        bottomGripper.update();
        topGripper.update();
    }

    @Override
    public void emergencyStop() {
        intake.emergencyStop();
        bottomGripper.emergencyStop();
        topGripper.emergencyStop();
    }
}
