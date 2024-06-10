package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Pivot;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group="zz")
public class PivotCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    Pivot pivot;

    StickyGamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);
        pivot = new Pivot(hardware, Pivot.State.ROTATED);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            if(gamepad.right_bumper){
                Pivot.index = (Pivot.index + 1) % 4;
            }
            if(gamepad.left_bumper ){
                Pivot.index = (((Pivot.index - 1) % 4)+4)%4;
            }

            pivot.update();
            gamepad.update();

//            robotModules.telemetry(telemetry);

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Pivot index", Pivot.index);
            loopTimer.reset();

            telemetry.update();
        }
    }
}
