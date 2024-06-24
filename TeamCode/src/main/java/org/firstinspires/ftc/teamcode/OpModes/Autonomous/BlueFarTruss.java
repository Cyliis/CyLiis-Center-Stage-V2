package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LogicNodes.Nodes.BlueDariaNodes;
import org.firstinspires.ftc.teamcode.LogicNodes.Nodes.BlueFarTrussNodes;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.BlueFarTrussPositions;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Vision.PropDetectionBlueFar;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Far Truss")
public class BlueFarTruss extends LinearOpMode {

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;
    RobotModules robotModules;

    BlueFarTrussNodes nodes;

    VisionPortal portal;
    PropDetectionBlueFar processor;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue, true);

        drive = new MecanumDrive(hardware, MecanumDrive.RunMode.Vector, false);

        robotModules = new RobotModules(hardware, drive);

        hardware.startThreads(this);
        Hardware.AUTO = true;

        processor = new PropDetectionBlueFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

        int detectionCase = 2;

        robotModules.bottomGripper.setState(BottomGripper.State.CLOSED);
        robotModules.topGripper.setState(TopGripper.State.OPEN);

        while(opModeInInit() && !isStopRequested()){
            hardware.update();
            detectionCase = processor.detection;

            robotModules.initUpdate();
//            robotModules.telemetry(telemetry);
            telemetry.addData("detection", detectionCase);
            telemetry.update();
        }
        portal.close();
        robotModules.dropDown.setState(DropDown.State.UP);

//        detectionCase = 1;
        nodes = new BlueFarTrussNodes(drive, robotModules, detectionCase);

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