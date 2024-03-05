package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class BuruDriveTrainControl implements IRobotModule {

    private final Gamepad gamepad;
    private final StickyGamepad stickyGamepad;
    private final MecanumDrive drive;

    public BuruDriveTrainControl(Gamepad gamepad, MecanumDrive drive){
        this.gamepad = gamepad;
        this.stickyGamepad = new StickyGamepad(gamepad);
        this.drive = drive;
    }

    double driveDeadZone = 0.05;

    @Override
    public void update() {
        if(gamepad.back) {
            drive.getLocalizer().imu.resetImu();
        }

        double forward = -gamepad.left_stick_y;
        if(Math.abs(forward) <= driveDeadZone) forward = 0;
        double strafe = -gamepad.left_stick_x;
        if(Math.abs(strafe) <= driveDeadZone) strafe = 0;

        drive.setTargetVector(new Vector(forward, strafe, gamepad.left_trigger - gamepad.right_trigger));

        stickyGamepad.update();
    }
}
