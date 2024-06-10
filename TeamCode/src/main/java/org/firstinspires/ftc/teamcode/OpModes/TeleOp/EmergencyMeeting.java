package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;

@TeleOp(name="🚨ඞ🚨ඞ🚨ඞ🚨ඞ🚨ඞ🚨", group="main")
public class EmergencyMeeting extends LinearOpMode {

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;
    RobotModules robotModules;

    BuruSebiGamepadControl gamepadControl;
    BuruDriveTrainControl driveTrainControl;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Universal);

        drive = new MecanumDrive(hardware, MecanumDrive.RunMode.Vector, true);

        robotModules = new RobotModules(hardware, drive);

        gamepadControl = new BuruSebiGamepadControl(robotModules, gamepad1, gamepad2);
        driveTrainControl = new BuruDriveTrainControl(gamepad1, drive);

        hardware.startThreads(this);

        robotModules.extension.setState(Extension.State.FAR);
        robotModules.extension.servo1.targetPosition = Extension.outPosition1;
        robotModules.extension.servo2.targetPosition = Extension.outPosition2;
        robotModules.extension.servo1.forceUpdate();
        robotModules.extension.servo2.forceUpdate();

        while(opModeInInit() && !isStopRequested()){
            robotModules.initUpdate();
            robotModules.telemetry(telemetry);
            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            gamepadControl.update();
            driveTrainControl.update();

            drive.update();

            robotModules.update();

            robotModules.telemetry(telemetry);

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}