package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

public class BuruSebiGamepadControl implements IRobotModule {

    private final RobotModules robotModules;
    private final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final Gamepad gamepad1, gamepad2;

    public BuruSebiGamepadControl(RobotModules robotModules, Gamepad gamepad1, Gamepad gamepad2){
        this.robotModules = robotModules;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    @Override
    public void update() {
        stickyGamepad1.update();
        stickyGamepad2.update();
    }
}
