package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.TopGripper;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Utils.UnStickyGamepad;

public class BuruSebiGamepadControl implements IRobotModule {

    private final RobotModules robotModules;
    private final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final UnStickyGamepad unStickyGamepad1, unStickyGamepad2;
    private final Gamepad gamepad1, gamepad2;

    public BuruSebiGamepadControl(RobotModules robotModules, Gamepad gamepad1, Gamepad gamepad2){
        this.robotModules = robotModules;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
        this.unStickyGamepad1 = new UnStickyGamepad(gamepad1);
        this.unStickyGamepad2 = new UnStickyGamepad(gamepad2);
        timer.startTime();
    }

    ElapsedTime timer = new ElapsedTime();

    public static double triggerThreshold = 0.1;

    public void updateIntake(){
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

    @Override
    public void update() {
        updateIntake();
        stickyGamepad1.update();
        stickyGamepad2.update();
        timer.reset();
    }
}
