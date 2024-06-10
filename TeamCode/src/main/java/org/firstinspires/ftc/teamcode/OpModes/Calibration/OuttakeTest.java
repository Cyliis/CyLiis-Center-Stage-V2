package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pivot;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group="zz")
public class OuttakeTest extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    Lift lift;
    Extension extension;
    Turret turret;
    Outtake outtake;
    Pivot pivot;

    TopGripper topGripper;
    BottomGripper bottomGripper;

    StickyGamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        gamepad = new StickyGamepad(gamepad1);

        hardware.startThreads(this);

        lift = new Lift(hardware, Lift.State.GOING_DOWN);
        extension = new Extension(hardware, Extension.State.IN);
        turret = new Turret(hardware, Turret.State.MIDDLE);
        pivot = new Pivot(hardware, Pivot.State.HOME);
        outtake = new Outtake(lift, extension, turret, pivot, Outtake.State.DOWN);

        topGripper = new TopGripper(hardware, TopGripper.State.OPEN);
        bottomGripper = new BottomGripper(hardware, BottomGripper.State.OPEN);

        while(opModeInInit() && !isStopRequested()){

            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            if(gamepad.x){
                if(outtake.getState() == Outtake.State.UP) outtake.setState(Outtake.State.GOING_DOWN);
                else if(outtake.getState() == Outtake.State.DOWN) outtake.setState(Outtake.State.GOING_UP_FAR);
            }

            if(gamepad.a){
                if(topGripper.getState() == TopGripper.State.OPEN){
                    topGripper.setState(TopGripper.State.CLOSING);
                    bottomGripper.setState(BottomGripper.State.CLOSING);
                }
                else if(topGripper.getState() == TopGripper.State.CLOSED){
                    topGripper.setState(TopGripper.State.OPENING);
                    bottomGripper.setState(BottomGripper.State.OPENING);
                }
            }

            if(gamepad.dpad_up){
                Lift.level = Math.min(11, Lift.level + 1);
            }
            if(gamepad.dpad_down){
                Lift.level = Math.max(0, Lift.level - 1);
            }

            outtake.update();
            bottomGripper.update();
            topGripper.update();
            gamepad.update();

//            robotModules.telemetry(telemetry);

            telemetry.addData("Outtake state", outtake.getState());
            telemetry.addData("Lift state", lift.getState());
            telemetry.addData("Extension state", extension.getState());
            telemetry.addData("Turret state", turret.getState());

//            while (loopTimer.seconds() <= 0.025){}

            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}
