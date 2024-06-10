package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group="zz")
public class IntakeCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    DropDown dropDown;

    StickyGamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);
        dropDown = new DropDown(hardware, DropDown.State.UP);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            if(gamepad.x){
                if(dropDown.getState() == DropDown.State.UP)dropDown.setState(DropDown.State.INTAKE);
                else if(dropDown.getState() == DropDown.State.INTAKE)dropDown.setState(DropDown.State.UP);
            }
            if(gamepad.left_bumper){
                DropDown.index = Math.max(0, DropDown.index - 1);
            }
            if(gamepad.right_bumper){
                DropDown.index = Math.min(4, DropDown.index + 1);
            }

            dropDown.update();
            gamepad.update();

            hardware.meh3.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

//            robotModules.telemetry(telemetry);

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Dropdown level", DropDown.index);
            telemetry.addData("Dropdown state", dropDown.getState());
            loopTimer.reset();

            telemetry.update();
        }
    }
}
