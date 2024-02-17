package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
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
    Encoder encoder;

    StickyGamepad gamepad;

    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);

        motor = new CoolMotor(hardware.mch3, CoolMotor.RunMode.PID, Extendo.motorReversed);

        encoder = hardware.ech2;
        if(Extendo.encoderReversed) encoder.setDirection(Encoder.Direction.REVERSE);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            motor.setMode(CoolMotor.RunMode.PID);
            motor.setPIDF(Extendo.pidf, Extendo.pidf.f * Math.signum(target - encoder.getCurrentPosition()));
            motor.calculatePower(encoder.getCurrentPosition(), target);

            motor.update();

            telemetry.addData("Target", target);
            telemetry.addData("Current", encoder.getCurrentPosition());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}

