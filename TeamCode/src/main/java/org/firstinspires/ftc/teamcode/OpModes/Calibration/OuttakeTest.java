package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    FtcDashboard dash;

    Hardware hardware;

    Lift lift;
    Extension extension;
    Turret turret;
    Outtake outtake;

    TopGripper topGripper;
    BottomGripper bottomGripper;

    MecanumDrive drive;
    BuruDriveTrainControl a;

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
        outtake = new Outtake(lift, extension, turret, Outtake.State.DOWN);

        topGripper = new TopGripper(hardware, TopGripper.State.OPEN);
        bottomGripper = new BottomGripper(hardware, BottomGripper.State.OPEN);

        drive = new MecanumDrive(hardware, MecanumDrive.RunMode.Vector, true);
        a = new BuruDriveTrainControl(gamepad1, drive);

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

            if(gamepad1.right_bumper) hardware.mch0.setPower(1);
            else if(gamepad1.left_bumper) hardware.mch0.setPower(-0.8);
            else hardware.mch0.setPower(0);

            a.update();
            drive.update();

            outtake.update();
            bottomGripper.update();
            topGripper.update();
            gamepad.update();

//            robotModules.telemetry(telemetry);

            hardware.update();
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
