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
@TeleOp(group="zz")
public class ExtendoCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    CoolMotor motor1, motor2;
    Encoder encoder;

    StickyGamepad gamepad;

    public static int target = 0;

    public static double hertz = 30;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Universal);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);

        motor1 = new CoolMotor(hardware.meh0, CoolMotor.RunMode.PID, Extendo.motor1Reversed);
        motor2 = new CoolMotor(hardware.meh1, CoolMotor.RunMode.PID, Extendo.motor2Reversed);
        encoder = hardware.eeh0;
        if(Extendo.encoderReversed) encoder.setDirection(Encoder.Direction.REVERSE);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            motor1.setPIDF(Extendo.PIDF, Extendo.PIDF.f * Math.signum(motor1.getPIDPower(encoder.getCurrentPosition(), target)));
            motor1.calculatePIDPower(encoder.getCurrentPosition(), target);
            motor2.setPIDF(Extendo.PIDF, Extendo.PIDF.f * Math.signum(motor2.getPIDPower(encoder.getCurrentPosition(), target)));
            motor2.calculatePIDPower(encoder.getCurrentPosition(), target);

            motor1.update();
            motor2.update();

            while (loopTimer.seconds() <= (1.0/hertz));

            telemetry.addData("Target", motor1.motor.motor.getTargetPosition());
            telemetry.addData("Current", encoder.getCurrentPosition());
            telemetry.addData("Voltage", Hardware.voltage);
            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}

