package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.DashboardUtil;

@TeleOp
public class LocalizationTest extends LinearOpMode {

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;
    RobotModules robotModules;

    BuruDriveTrainControl driveTrainControl;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        drive = new MecanumDrive(hardware, MecanumDrive.RunMode.Vector, false);

        robotModules = new RobotModules(hardware, drive);

        driveTrainControl = new BuruDriveTrainControl(gamepad1, drive);

        hardware.startThreads(this);

        while(opModeInInit() && !isStopRequested()){
            robotModules.initUpdate();
//            robotModules.telemetry(telemetry);
            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            if(robotModules.extendo.getState() != Extendo.State.OUT) robotModules.extendo.setState(Extendo.State.OUT);

            driveTrainControl.update();

            drive.update();

            robotModules.update();

//            robotModules.telemetry(telemetry);

            telemetry.addData("Pose X",hardware.localizer.getPoseEstimate().getX());
            telemetry.addData("Pose Y",hardware.localizer.getPoseEstimate().getY());
            telemetry.addData("Heading",hardware.localizer.getPoseEstimate().getHeading());

            telemetry.addData("extendo pos", robotModules.extendo.motor.getCurrentPosition() - Extendo.zeroPos);

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }


}