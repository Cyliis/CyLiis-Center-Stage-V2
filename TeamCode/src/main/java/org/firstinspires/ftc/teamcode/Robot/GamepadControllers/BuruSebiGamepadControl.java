package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Other.Hooks;
import org.firstinspires.ftc.teamcode.Modules.Other.PTOs;
import org.firstinspires.ftc.teamcode.Modules.Other.Plane;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pivot;
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

    public static double triggerThreshold = 0.3;
    public static double extensionDeadZone = 0.1;
    public static double timeoutThreshold = 0.7;
    public static double addedTimeOut = 0.1;

    public void updateIntake(){
        if(!Intake.ENABLED) return;
        if(stickyGamepad2.right_bumper){
            DropDown.index = 3;
        }
        if(stickyGamepad2.left_bumper){
            DropDown.index = 0;
        }
        if(Math.abs(gamepad1.right_stick_y) > extensionDeadZone && gamepad1.right_stick_y < 0
                && robotModules.intake.getState()!= Intake.State.GOING_IN){ // extendo out
            if(robotModules.bottomGripper.getState() == BottomGripper.State.OPEN &&
                    robotModules.topGripper.getState() == TopGripper.State.OPEN){ // can go out
                robotModules.extendo.setState(Extendo.State.OUT);
                if(robotModules.extendo.encoder.getCurrentPosition() - Extendo.zeroPos < Extendo.extensionLimit)
                    robotModules.extendo.setPower(-gamepad1.right_stick_y);
                else robotModules.extendo.setPower(0);
            }
            if(robotModules.topGripper.getState() == TopGripper.State.CLOSED || robotModules.topGripper.getState() == TopGripper.State.CLOSING){
                robotModules.topGripper.setState(TopGripper.State.OPENING);
                robotModules.extendo.setState(Extendo.State.GOING_OUT);
            }
            if(robotModules.bottomGripper.getState() == BottomGripper.State.CLOSED || robotModules.bottomGripper.getState() == BottomGripper.State.CLOSING){
                robotModules.bottomGripper.setState(BottomGripper.State.OPENING);
                robotModules.extendo.setState(Extendo.State.GOING_OUT);
            }
            if(robotModules.intake.getState() == Intake.State.CLOSING_GRIPPERS){
                robotModules.intake.setState(Intake.State.OPENING_GRIPPERS);
                robotModules.extendo.setState(Extendo.State.GOING_OUT);
            }
        }
        if(Math.abs(gamepad1.right_stick_y) > extensionDeadZone && gamepad1.right_stick_y > 0){ // extendo in
            if(robotModules.extendo.encoder.getCurrentPosition() - Extendo.zeroPos > 0) {
                robotModules.extendo.setPower(-gamepad1.right_stick_y);
                if(robotModules.extendo.getState() == Extendo.State.OUT && Math.abs(gamepad1.right_stick_y) < timeoutThreshold)
                    Extendo.timeOut = addedTimeOut;
            }
            else robotModules.extendo.setPower(0);
            if(robotModules.extendo.encoder.getCurrentPosition() - Extendo.zeroPos <= Extendo.inThreshold
                    && (robotModules.extendo.getState() == Extendo.State.OUT || robotModules.extendo.getState() == Extendo.State.GOING_OUT))
                robotModules.intake.setState(Intake.State.GOING_IN);
        }
        if(Math.abs(gamepad1.right_stick_y) <= extensionDeadZone) { //extendo idle
            if(robotModules.extendo.encoder.getCurrentPosition() - Extendo.zeroPos <= Extendo.inThreshold
                    && (robotModules.extendo.getState() == Extendo.State.OUT || robotModules.extendo.getState() == Extendo.State.GOING_OUT))
                robotModules.intake.setState(Intake.State.GOING_IN);
            robotModules.extendo.setPower(0); // extendo idle
        }
//        if(stickyGamepad2.b){
//            if(robotModules.extendo.getState() == Extendo.State.OUT)
//                robotModules.intake.setState(Intake.State.GOING_IN);
//        }
        if(robotModules.outtake.getState() != Outtake.State.DOWN) {
            if(gamepad2.left_trigger >= triggerThreshold || gamepad1.left_bumper) robotModules.activeIntake.setState(ActiveIntake.State.REVERSE);
            else robotModules.activeIntake.setState(ActiveIntake.State.IDLE);
            return;
        }
        if(gamepad2.left_trigger >= triggerThreshold){ // press to reverse
            if(robotModules.intake.getState() == Intake.State.IDLE || robotModules.intake.getState() == Intake.State.OPENING_GRIPPERS
            || robotModules.intake.getState() == Intake.State.INTAKE || robotModules.intake.getState() == Intake.State.GOING_IN
            || robotModules.intake.getState() == Intake.State.CLOSING_GRIPPERS || robotModules.intake.getState() == Intake.State.AUTO_REVERSE){
                if(robotModules.activeIntake.getState() != ActiveIntake.State.REVERSE) robotModules.intake.setState(Intake.State.REVERSE);
                if(robotModules.extendo.getState() == Extendo.State.IN)
                    if(robotModules.bottomGripper.getState() == BottomGripper.State.OPEN || robotModules.bottomGripper.getState() == BottomGripper.State.OPENING
                    || robotModules.topGripper.getState() == TopGripper.State.OPEN || robotModules.topGripper.getState() == TopGripper.State.OPENING)
                        robotModules.intake.setState(Intake.State.CLOSING_GRIPPERS);
            }
        }
        else if(gamepad2.right_trigger >= triggerThreshold){ // press to intake
            if(robotModules.intake.getState() == Intake.State.IDLE || robotModules.intake.getState() == Intake.State.CLOSING_GRIPPERS
                    || robotModules.intake.getState() == Intake.State.REVERSE)
                robotModules.intake.setState(Intake.State.START_INTAKE);
            //might mess up
        } else { // idle
            if(robotModules.intake.getState() == Intake.State.OPENING_GRIPPERS || robotModules.intake.getState() == Intake.State.INTAKE
            || robotModules.intake.getState() == Intake.State.REVERSE || robotModules.intake.getState() == Intake.State.AUTO_REVERSE)
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
            Lift.level = Math.min(7.5, Lift.level + 0.5);
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.CHANGING_LIFT_POSITION);
        }
        if(stickyGamepad2.dpad_down){
            Lift.level = Math.max(-2, Lift.level - 0.5);
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.CHANGING_LIFT_POSITION);
        }
        if(stickyGamepad1.dpad_down){
            Lift.level = Math.max(-2, Lift.level - 0.5);
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.CHANGING_LIFT_POSITION);
        }
        if(stickyGamepad1.dpad_up){
            Lift.level = Math.min(7.5, Lift.level + 0.5);
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.CHANGING_LIFT_POSITION);
        }
//        if(robotModules.intake.getState() != Intake.State.IDLE) return;

        if(robotModules.lift.getState() == Lift.State.GOING_DOWN){
            Lift.level = Math.ceil(Lift.level);
        }

        if(Extendo.ENABLED && robotModules.extendo.getState() != Extendo.State.IN) return;

        if(stickyGamepad2.x){
            if(robotModules.outtake.getState() == Outtake.State.UP && (robotModules.extension.getState() == Extension.State.GOING_CLOSE || robotModules.extension.getState() == Extension.State.CLOSE)) robotModules.extension.setState(Extension.State.GOING_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.POKE) robotModules.outtake.setState(Outtake.State.EXTEND_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.DOWN &&
                    (robotModules.topGripper.getState() == TopGripper.State.CLOSED  && robotModules.bottomGripper.getState() == BottomGripper.State.CLOSED))
                robotModules.outtake.setState(Outtake.State.GOING_UP_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.EXTEND_CLOSE1) robotModules.outtake.setState(Outtake.State.EXTEND_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.GOING_UP_CLOSE) robotModules.outtake.setState(Outtake.State.GOING_UP_FAR);
            else if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION) {
                if((robotModules.topGripper.getState() == TopGripper.State.OPEN || robotModules.topGripper.getState() == TopGripper.State.OPENING)
                        && (robotModules.bottomGripper.getState() == BottomGripper.State.OPEN || robotModules.bottomGripper.getState() == BottomGripper.State.OPENING))
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
            }
        }
        if(stickyGamepad2.a){
            if(robotModules.outtake.getState() == Outtake.State.UP && (robotModules.extension.getState() == Extension.State.GOING_FAR || robotModules.extension.getState() == Extension.State.FAR)) robotModules.extension.setState(Extension.State.GOING_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.POKE) robotModules.outtake.setState(Outtake.State.EXTEND_CLOSE1);
            else if(robotModules.outtake.getState() == Outtake.State.DOWN) robotModules.outtake.setState(Outtake.State.GOING_UP_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.EXTEND_FAR) robotModules.outtake.setState(Outtake.State.EXTEND_CLOSE1);
            else if(robotModules.outtake.getState() == Outtake.State.GOING_UP_FAR) robotModules.outtake.setState(Outtake.State.GOING_UP_CLOSE);
            else if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION) {
                if((robotModules.topGripper.getState() == TopGripper.State.OPEN || robotModules.topGripper.getState() == TopGripper.State.OPENING)
                        && (robotModules.bottomGripper.getState() == BottomGripper.State.OPEN || robotModules.bottomGripper.getState() == BottomGripper.State.OPENING))
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
            }
        }
        if(stickyGamepad2.b){
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION
            || robotModules.outtake.getState() == Outtake.State.GOING_UP_CLOSE || robotModules.outtake.getState() == Outtake.State.GOING_UP_FAR
                    || robotModules.outtake.getState() == Outtake.State.EXTEND_CLOSE1 || robotModules.outtake.getState() == Outtake.State.EXTEND_FAR ||
            robotModules.outtake.getState() == Outtake.State.EXTEND_POKE || robotModules.outtake.getState() == Outtake.State.GOING_POKE ||
            robotModules.outtake.getState() == Outtake.State.GO_POKE_FROM_UP || robotModules.outtake.getState() == Outtake.State.POKE){
                robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                if(robotModules.topGripper.getState() != TopGripper.State.OPEN)
                    robotModules.topGripper.setState(TopGripper.State.OPENING);
                if(robotModules.bottomGripper.getState() != BottomGripper.State.OPEN)
                    robotModules.bottomGripper.setState(BottomGripper.State.OPENING);
            }
        }
        if(stickyGamepad2.dpad_right){
            if(robotModules.outtake.getState() == Outtake.State.DOWN) robotModules.outtake.setState(Outtake.State.GOING_POKE);
            if(robotModules.outtake.getState() == Outtake.State.POKE) {
                if((robotModules.bottomGripper.getState() == BottomGripper.State.OPEN || robotModules.bottomGripper.getState() == BottomGripper.State.OPENING) &&
                        (robotModules.topGripper.getState() == TopGripper.State.OPEN || robotModules.topGripper.getState() == TopGripper.State.OPENING))
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
            }
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.GO_POKE_FROM_UP);
        }
        if(stickyGamepad1.left_bumper) Pivot.index = (((Pivot.index - 1) % 4) + 4) % 4;
        if(stickyGamepad1.right_bumper) Pivot.index = (Pivot.index + 1) % 4;
    }

    public void updatePlane(){
        if(!Plane.ENABLED) return;
        if(doubleStickyGamepad.y && robotModules.plane.getState() == Plane.State.CLOSED)
            robotModules.plane.setState(Plane.State.OPEN);
    }

    public void updateClimb(){
        if(!PTOs.ENABLED || !Hooks.ENABLED) return;
        if(doubleStickyGamepad.y && (robotModules.plane.getState() == Plane.State.OPEN || !Plane.ENABLED)){
            robotModules.hooks.setState(Hooks.State.OPEN);
        }
        if(gamepad2.y && stickyGamepad1.ps){
            if(robotModules.ptos.getState() == PTOs.State.DISENGAGED) {
                robotModules.ptos.setState(PTOs.State.ENGAGED);
                robotModules.drive.setRunMode(MecanumDrive.RunMode.Climb);
            }
            else {
                robotModules.ptos.setState(PTOs.State.DISENGAGED);
                robotModules.drive.setRunMode(MecanumDrive.RunMode.Vector);
            }
        }
    }

    void updateGrippers(){
        if(!TopGripper.ENABLED || !BottomGripper.ENABLED) return;
        if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.CHANGING_LIFT_POSITION ||
        robotModules.outtake.getState() == Outtake.State.LIFT_GOING_UP || robotModules.outtake.getState() == Outtake.State.POKE){

            boolean both = false, bottom = false, top = false;

            if(Pivot.index == 1){
                both = stickyGamepad1.x || stickyGamepad1.b;
                bottom = stickyGamepad1.a;
                top = stickyGamepad1.y;
            }
            else if(Pivot.index == 0){
                bottom = stickyGamepad1.b;
                top = stickyGamepad1.x;
                both = stickyGamepad1.a || stickyGamepad1.y;
            }
            else if(Pivot.index == 2){
                bottom = stickyGamepad1.x;
                top = stickyGamepad1.b;
                both = stickyGamepad1.a || stickyGamepad1.y;
            } else if(Pivot.index == 3){
                both = stickyGamepad1.x || stickyGamepad1.b;
                bottom = stickyGamepad1.y;
                top = stickyGamepad1.a;
            }

            if(both){
                if(robotModules.topGripper.getState() == TopGripper.State.CLOSED) robotModules.topGripper.setState(TopGripper.State.OPENING);
                if(robotModules.bottomGripper.getState() == BottomGripper.State.CLOSED) robotModules.bottomGripper.setState(BottomGripper.State.OPENING);
            }
            if(bottom){
                if(robotModules.bottomGripper.getState() == BottomGripper.State.CLOSED) robotModules.bottomGripper.setState(BottomGripper.State.OPENING);
            }
            if(top){
                if(robotModules.topGripper.getState() == TopGripper.State.CLOSED) robotModules.topGripper.setState(TopGripper.State.OPENING);
            }
        }
    }

    boolean detected0, detected1;

    void beamBreakLogic(){
        boolean newDetection0 = !robotModules.beamBreak0.getState();
        boolean newDetection1 = !robotModules.beamBreak1.getState();

        if(!detected0 && newDetection0 && robotModules.intake.getState() == Intake.State.INTAKE){
            gamepad1.rumble(0,1,500);
            gamepad2.rumble(0,1,500);
        }if(!detected1 && newDetection1 && robotModules.intake.getState() == Intake.State.INTAKE){
            gamepad1.rumble(1,0,500);
            gamepad2.rumble(1,0,500);
        }

        detected0 = newDetection0;
        detected1 = newDetection1;

    }

    @Override
    public void update() {
        updateIntake();
        updateOuttake();
        updateClimb();
        updatePlane();
        updateGrippers();
        beamBreakLogic();
        stickyGamepad1.update();
        stickyGamepad2.update();
        unStickyGamepad1.update();
        unStickyGamepad2.update();
        doubleStickyGamepad.update();
        timer.reset();
    }
}
