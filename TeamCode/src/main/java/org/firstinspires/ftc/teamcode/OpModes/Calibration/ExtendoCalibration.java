package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
@TeleOp
public class ExtendoCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    CoolMotor motor;

    StickyGamepad gamepad;

    public static int target = 0;

    public static double hertz = 50;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);

        motor = new CoolMotor(hardware.meh0, CoolMotor.RunMode.RTP, Extendo.motorReversed);

        motor.setPower(1);
        motor.setTarget(0);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            motor.setPIDF(Extendo.PIDF);
            motor.setTarget(target);

            motor.update();

            while (loopTimer.seconds() <= (1.0/hertz));

            telemetry.addData("Target", motor.motor.motor.getTargetPosition());
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.addData("Voltage", Hardware.voltage);
            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}

