package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
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
        turret = new Turret(hardware, Turret.State.GOING_MIDDLE);
        outtake = new Outtake(lift, extension, turret, Outtake.State.DOWN);

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

            outtake.update();
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
