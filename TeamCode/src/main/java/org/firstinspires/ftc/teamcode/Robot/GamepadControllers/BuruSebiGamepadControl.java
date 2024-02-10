package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Other.Climb;
import org.firstinspires.ftc.teamcode.Modules.Other.Plane;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.DoubleStickyGamepad;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Utils.UnStickyGamepad;

public class BuruSebiGamepadControl implements IRobotModule {

    private final RobotModules robotModules;
    private final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final UnStickyGamepad unStickyGamepad1, unStickyGamepad2;
    private final DoubleStickyGamepad doubleStickyGamepad;
    private final Gamepad gamepad1, gamepad2;

    public BuruSebiGamepadControl(RobotModules robotModules, Gamepad gamepad1, Gamepad gamepad2){
        this.robotModules = robotModules;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
        this.unStickyGamepad1 = new UnStickyGamepad(gamepad1);
        this.unStickyGamepad2 = new UnStickyGamepad(gamepad2);
        this.doubleStickyGamepad = new DoubleStickyGamepad(gamepad1, gamepad2);
        timer.startTime();
    }

    ElapsedTime timer = new ElapsedTime();

    public static double triggerThreshold = 0.1;

    public void updateIntake(){
        if(!Intake.ENABLED) return;
        if(robotModules.outtake.getState() != Outtake.State.DOWN) {
            if(gamepad2.left_trigger >= triggerThreshold || gamepad1.left_bumper) robotModules.activeIntake.setState(ActiveIntake.State.REVERSE);
            else robotModules.activeIntake.setState(ActiveIntake.State.IDLE);
            return;
        }
        if(gamepad1.right_bumper){ // extendo out
            if(robotModules.bottomGripper.getState() == BottomGripper.State.OPEN &&
            robotModules.topGripper.getState() == TopGripper.State.OPEN){ // can go out
                Extendo.extendedPos = (int)Math.min(Extendo.extensionLimit,
                        Extendo.extendedPos + (int)(timer.seconds() * Extendo.extensionRate));
            }
        }
        if(gamepad1.left_bumper){ // extendo in
            Extendo.extendedPos = (int)Math.max(0, Extendo.extendedPos - (int)(timer.seconds() * Extendo.extensionRate));
            if(Extendo.extendedPos == 0) robotModules.extendo.setState(Extendo.State.GOING_IN);
        }
        if(stickyGamepad1.b && // extendo completly in
                (robotModules.extendo.getState() == Extendo.State.OUT || robotModules.extendo.getState() == Extendo.State.GOING_OUT)){
            robotModules.intake.setState(Intake.State.GOING_IN);
        }
        if(gamepad2.left_trigger >= triggerThreshold){ // press to reverse
            if(robotModules.intake.getState() == Intake.State.IDLE || robotModules.intake.getState() == Intake.State.OPENING_GRIPPERS
            || robotModules.intake.getState() == Intake.State.INTAKE || robotModules.intake.getState() == Intake.State.GOING_IN
            || robotModules.intake.getState() == Intake.State.CLOSING_GRIPPERS){
                if(robotModules.activeIntake.getState() != ActiveIntake.State.REVERSE) robotModules.intake.setState(Intake.State.REVERSE);
                if(robotModules.extendo.getState() == Extendo.State.IN)
                    if(robotModules.bottomGripper.getState() == BottomGripper.State.OPEN || robotModules.bottomGripper.getState() == BottomGripper.State.OPENING
                    || robotModules.topGripper.getState() == TopGripper.State.OPEN || robotModules.topGripper.getState() == TopGripper.State.OPENING)
                        robotModules.intake.setState(Intake.State.CLOSING_GRIPPERS);
            }
        }
        else if(gamepad2.right_trigger >= triggerThreshold){ // press to intake
            if(robotModules.intake.getState() == Intake.State.IDLE || robotModules.intake.getState() == Intake.State.CLOSING_GRIPPERS)
                robotModules.intake.setState(Intake.State.START_INTAKE);
        } else { // idle
            if(robotModules.intake.getState() == Intake.State.OPENING_GRIPPERS || robotModules.intake.getState() == Intake.State.INTAKE
            || robotModules.intake.getState() == Intake.State.REVERSE || robotModules.intake.getState() == Intake.State.GOING_IN)
                robotModules.intake.setState(Intake.State.STOP_INTAKE);
            if(robotModules.extendo.getState() == Extendo.State.IN)
                if(robotModules.bottomGripper.getState() == BottomGripper.State.OPEN || robotModules.bottomGripper.getState() == BottomGripper.State.OPENING
                        || robotModules.topGripper.getState() == TopGripper.State.OPEN || robotModules.topGripper.getState() == TopGripper.State.OPENING)
                    robotModules.intake.setState(Intake.State.CLOSING_GRIPPERS);
        }

    }

    public void updateOuttake(){
        if(!Outtake.ENABLED) return;
        if(stickyGamepad2.dpad_up){
            Lift.level = Math.min(8, Lift.level + 1);
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION) robotModules.outtake.setState(Outtake.State.CHANGING_LIFT_POSITION);
        }
        if(stickyGamepad2.dpad_down){
            Lift.level = Math.max(0, Lift.level - 1);
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION) robotModules.outtake.setState(Outtake.State.CHANGING_LIFT_POSITION);
        }
//        if(robotModules.intake.getState() != Intake.State.IDLE) return;
        if(stickyGamepad2.x){
            if(robotModules.outtake.getState() == Outtake.State.UP && (robotModules.extension.getState() == Extension.State.GO_CLOSE || robotModules.extension.getState() == Extension.State.CLOSE)) robotModules.extension.setState(Extension.State.GO_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.DOWN) robotModules.outtake.setState(Outtake.State.GOING_UP_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.EXTEND_CLOSE) robotModules.outtake.setState(Outtake.State.EXTEND_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.GOING_UP_CLOSE) robotModules.outtake.setState(Outtake.State.GOING_UP_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION) robotModules.outtake.setState(Outtake.State.GOING_DOWN);
        }
        if(stickyGamepad2.a){
            if(robotModules.outtake.getState() == Outtake.State.UP && (robotModules.extension.getState() == Extension.State.GO_FAR || robotModules.extension.getState() == Extension.State.FAR)) robotModules.extension.setState(Extension.State.GO_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.DOWN) robotModules.outtake.setState(Outtake.State.GOING_UP_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.EXTEND_FAR) robotModules.outtake.setState(Outtake.State.EXTEND_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.GOING_UP_FAR) robotModules.outtake.setState(Outtake.State.GOING_UP_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION) robotModules.outtake.setState(Outtake.State.GOING_DOWN);
        }
    }

    public void updatePlane(){
        if(!Plane.ENABLED) return;
        if(doubleStickyGamepad.y && robotModules.plane.getState() == Plane.State.CLOSED)
            robotModules.plane.setState(Plane.State.OPEN);
    }

    public void updateClimb(){
        if(!Climb.ENABLED) return;
        if(doubleStickyGamepad.y && robotModules.plane.getState() == Plane.State.OPEN){
            if(robotModules.climb.getState() == Climb.State.DISENGAGED) robotModules.climb.setState(Climb.State.HOOKS_DEPLOYED);
            else if(robotModules.climb.getState() == Climb.State.HOOKS_DEPLOYED) {
                robotModules.climb.setState(Climb.State.ENGAGED);
                robotModules.drive.setRunMode(MecanumDrive.RunMode.Climb);
            }
            else if(robotModules.climb.getState() == Climb.State.ENGAGED) {
                robotModules.climb.setState(Climb.State.HOOKS_DEPLOYED);
                robotModules.drive.setRunMode(MecanumDrive.RunMode.Vector);
            }
        }
    }

    @Override
    public void update() {
        updateIntake();
        updateOuttake();
        updatePlane();
        updateClimb();
        stickyGamepad1.update();
        stickyGamepad2.update();
        timer.reset();
    }
}
