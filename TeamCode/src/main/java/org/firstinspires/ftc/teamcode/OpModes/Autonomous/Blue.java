package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.Nodes.BlueNodes;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;

@Autonomous(name = "Denis albastru")
public class Blue extends LinearOpMode {

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;
    RobotModules robotModules;

    BlueNodes nodes;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue, true);

        drive = new MecanumDrive(hardware, MecanumDrive.RunMode.Vector, true);

        robotModules = new RobotModules(hardware, drive);

        nodes = new BlueNodes(drive, robotModules, 1);

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

            nodes.detector.update();
            nodes.currentNode.run();

            drive.update();

            robotModules.update();
            robotModules.telemetry(telemetry);

            telemetry.addData("Node", nodes.currentNode);
            telemetry.addData("Pose", drive.getLocalizer().getPoseEstimate());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}
