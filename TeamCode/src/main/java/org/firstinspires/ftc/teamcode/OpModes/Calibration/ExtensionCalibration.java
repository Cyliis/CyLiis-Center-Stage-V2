package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp
public class ExtensionCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    Extension extension;

    StickyGamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);
        extension = new Extension(hardware, Extension.State.IN);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            if(gamepad.x){
                if(extension.getState() == Extension.State.IN || extension.getState() == Extension.State.GOING_IN)
                    extension.setState(Extension.State.GOING_FAR);
                else if(extension.getState() == Extension.State.FAR || extension.getState() == Extension.State.GOING_FAR)
                    extension.setState(Extension.State.GOING_IN);
            }

            extension.update();
            gamepad.update();

//            robotModules.telemetry(telemetry);

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Extension state", extension.getState());
            telemetry.addLine("Press X to toggle state (IN/FAR)");
            loopTimer.reset();

            telemetry.update();
        }
    }
}
