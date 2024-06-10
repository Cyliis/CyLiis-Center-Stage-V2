package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group="zz")
public class SimpleDT extends OpMode {
    DcMotorEx mfl, mfr, mbl, mbr;

    @Override
    public void init() {
        mfl = hardwareMap.get(DcMotorEx.class, "eh3");
        mfr = hardwareMap.get(DcMotorEx.class, "eh0");
        mbl = hardwareMap.get(DcMotorEx.class, "eh2");
        mbr = hardwareMap.get(DcMotorEx.class, "eh1");

        mfr.setDirection(DcMotorSimple.Direction.REVERSE);
        mbr.setDirection(DcMotorSimple.Direction.REVERSE);

        mfl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mbl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mbr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotation = gamepad1.right_trigger - gamepad1.left_trigger;

        double denominator = Math.max(1, Math.abs(forward) + Math.abs(right) + Math.abs(rotation));

        forward/=denominator;
        right/=denominator;
        rotation/=denominator;

        mfl.setPower(forward - right + rotation);
        mbl.setPower(forward + right + rotation);
        mfr.setPower(forward - right - rotation);
        mbr.setPower(forward + right - rotation);

        telemetry.addData("front left current draw", mfl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("front right current draw", mfr.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("back left current draw", mbl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("back right current draw", mbr.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }
}
