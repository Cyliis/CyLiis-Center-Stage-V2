package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Disabled
@Config
@TeleOp
public class LiftCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    CoolMotor leftMotor, rightMotor;
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

        leftMotor = new CoolMotor(hardware.mch2, CoolMotor.RunMode.PID, Lift.leftMotorReversed);
        rightMotor = new CoolMotor(hardware.mch1, CoolMotor.RunMode.PID, Lift.rightMotorReversed);

        encoder = hardware.ech1;
        if(Lift.encoderReversed) encoder.setDirection(Encoder.Direction.REVERSE);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            leftMotor.setMode(CoolMotor.RunMode.PID);
            rightMotor.setMode(CoolMotor.RunMode.PID);
            leftMotor.setPIDF(Lift.pid, Lift.ff1 + Lift.ff2 * target);
            rightMotor.setPIDF(Lift.pid, Lift.ff1 + Lift.ff2 * target);
            leftMotor.calculatePower(encoder.getCurrentPosition(), target);
            rightMotor.calculatePower(encoder.getCurrentPosition(), target);

            leftMotor.update();
            rightMotor.update();

            telemetry.addData("Target", target);
            telemetry.addData("Current", encoder.getCurrentPosition());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}
