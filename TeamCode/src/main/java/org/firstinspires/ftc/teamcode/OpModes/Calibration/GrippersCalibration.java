package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp
public class GrippersCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    TopGripper topGripper;
    BottomGripper bottomGripper;

    StickyGamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);
        topGripper = new TopGripper(hardware, TopGripper.State.CLOSED);
        bottomGripper = new BottomGripper(hardware, BottomGripper.State.CLOSED);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            if(gamepad.x){
                if(topGripper.getState() == TopGripper.State.OPEN) {
                    topGripper.setState(TopGripper.State.CLOSING);
                    bottomGripper.setState(BottomGripper.State.CLOSING);
                }
                else if(topGripper.getState() == TopGripper.State.CLOSED){
                    topGripper.setState(TopGripper.State.OPENING);
                    bottomGripper.setState(BottomGripper.State.OPENING);
                }
            }

            topGripper.update();
            bottomGripper.update();
            gamepad.update();

//            robotModules.telemetry(telemetry);

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Top gripper state", topGripper.getState());
            telemetry.addData("Bottom gripper state", bottomGripper.getState());
            telemetry.addData("Bottom timer", bottomGripper.timer.seconds());
            telemetry.addData("Top timer", topGripper.timer.seconds());
            telemetry.addLine("Press X to toggle state (OPEN/CLOSED)");
            loopTimer.reset();

            telemetry.update();
        }
    }
}
