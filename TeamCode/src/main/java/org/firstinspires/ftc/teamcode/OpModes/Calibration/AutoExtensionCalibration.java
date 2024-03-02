package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Disabled
@TeleOp
public class AutoExtensionCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    Extension extension;
    Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        hardware.startThreads(this);
        extension = new Extension(hardware, Extension.State.BACKDROP);
        turret = new Turret(hardware, Turret.State.BACKDROP);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            extension.update();
            turret.update();

//            robotModules.telemetry(telemetry);


            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Imu angle", hardware.localizer.getHeading());
            telemetry.addData("Zero extension angle", extension.calculateAngle(0));
            telemetry.addData("Sensor reading", extension.sensorReading);
            telemetry.addData("Calculated extension", extension.calculateBackdropExtension());
            telemetry.addData("Calculated angle", extension.calculateAngle(extension.calculateBackdropExtension()));
            telemetry.addData("Calculated servo position", extension.getBackdropPosition());
            telemetry.addData("Angle delta", extension.calculateAngle(extension.calculateBackdropExtension()) - extension.calculateAngle(0));
            telemetry.addData("Servo delta", extension.servoDelta);
            telemetry.addData("Extension Vector", extension.extensionVector);
            telemetry.addData("Final position", Extension.State.BACKDROP.position2);
            telemetry.addData("Range", (extension.calculateAngle(Extension.limit) - extension.calculateAngle(0)/(Extension.outPosition2 - Extension.inPosition2)));

            loopTimer.reset();

            telemetry.update();
        }
    }
}