package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
@TeleOp
public class LiftCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    CoolMotor leftMotor, rightMotor;

    public static int target = 0;
    public static double hertz = 50;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        hardware.startThreads(this);

        leftMotor = new CoolMotor(hardware.meh1, CoolMotor.RunMode.RTP, Lift.leftMotorReversed);
        rightMotor = new CoolMotor(hardware.meh2, CoolMotor.RunMode.RTP, Lift.rightMotorReversed);

        leftMotor.setPower(Lift.power);
        rightMotor.setPower(Lift.power);

        leftMotor.motor.motor.setTargetPositionTolerance(20);

        leftMotor.setTarget(0);
        rightMotor.setTarget(0);

        while(opModeInInit() && !isStopRequested()){
            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

//            leftMotor.setMode(CoolMotor.RunMode.PID);
//            rightMotor.setMode(CoolMotor.RunMode.PID);
//            leftMotor.setPIDF(Lift.pid, Lift.ff1 + Lift.ff2 * target);
//            rightMotor.setPIDF(Lift.pid, Lift.ff1 + Lift.ff2 * target);
//            leftMotor.calculatePIDPower(encoder.getCurrentPosition(), target);
//            rightMotor.calculatePIDPower(encoder.getCurrentPosition(), target);

            leftMotor.setPIDF(Lift.pidf);
            rightMotor.setPIDF(Lift.pidf);

            leftMotor.setTarget(target);
            rightMotor.setTarget(target);

            leftMotor.update();
            rightMotor.update();

            while (loopTimer.seconds() <= (1.0/hertz));

            telemetry.addData("Target", leftMotor.motor.motor.getTargetPosition());
            telemetry.addData("P", leftMotor.motor.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
            telemetry.addData("I", leftMotor.motor.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
            telemetry.addData("D", leftMotor.motor.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
            telemetry.addData("F", leftMotor.motor.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f);
            telemetry.addData("Current", leftMotor.getCurrentPosition());
            telemetry.addData("Current", rightMotor.getCurrentPosition());
            telemetry.addData("Voltage", Hardware.voltage);
            telemetry.addData("Power", leftMotor.motor.motor.getPower());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}
