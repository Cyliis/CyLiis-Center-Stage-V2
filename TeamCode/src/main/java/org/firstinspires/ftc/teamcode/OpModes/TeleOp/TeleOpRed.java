package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red 🐟")
public class TeleOpRed extends LinearOpMode {

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

        hardware = new Hardware(hardwareMap, Hardware.Color.Red);

        drive = new MecanumDrive(hardware, MecanumDrive.RunMode.Vector, hardware.localizer, true);

        robotModules = new RobotModules(hardware, drive);

        gamepadControl = new BuruSebiGamepadControl(robotModules, gamepad1, gamepad2);
        driveTrainControl = new BuruDriveTrainControl(gamepad1, drive);

        hardware.startThreads(this);

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

//            robotModules.telemetry(telemetry);

            hardware.update();

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}