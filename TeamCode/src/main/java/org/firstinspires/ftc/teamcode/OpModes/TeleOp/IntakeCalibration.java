package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp
public class IntakeCalibration extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    Ramp ramp;
    DropDown dropDown;

    StickyGamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);
        ramp = new Ramp(hardware, Ramp.State.INTAKE);
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
                ramp.setState(Ramp.State.INTAKE);
                dropDown.setState(DropDown.State.INTAKE);
            }
            if(gamepad.a){
                ramp.setState(Ramp.State.UP);
                dropDown.setState(DropDown.State.UP);
            }
            if(gamepad.left_bumper){
                Ramp.index = Math.max(0, Ramp.index - 1);
                DropDown.index = Math.max(0, DropDown.index - 1);
            }
            if(gamepad.right_bumper){
                Ramp.index = Math.min(4, Ramp.index + 1);
                DropDown.index = Math.min(4, DropDown.index + 1);
            }

            ramp.update();
            dropDown.update();
            gamepad.update();

//            robotModules.telemetry(telemetry);

            hardware.update();

            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Ramp level", Ramp.index);
            telemetry.addData("Dropdown level", DropDown.index);
            telemetry.addData("Ramp state", ramp.getState());
            telemetry.addData("Dropdown state", dropDown.getState());
            loopTimer.reset();

            telemetry.update();
        }
    }
}
