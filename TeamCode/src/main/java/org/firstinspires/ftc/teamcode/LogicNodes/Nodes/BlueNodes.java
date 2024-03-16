package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.BluePositions;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.DepositPixelDetector;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;

@Config
public class BlueNodes {

    private Pose purplePosition;
    private int preloadIntakeExtendoPosition;

    private Pose alignToCrossFieldForYellowPosition;
    private Pose crossFieldYellowPosition;
    private Pose scoreYellowPosition;
    private Pose scoreWhitePosition;

    private final double yellowCycleTime = 8;

    public static double fallTime = 0.5;

    public int cycle = -1;

    private Pose alignToCrossBackPosition;
    private Pose[] intakePositions;
    public static double[] extendoDistanceTolerance;
    public static double[] extendoHeadingTolerance;
    private int[] extendoPositions;
    private Pose[] scorePositions;
    public static double outtakeActivationLine = 60;
    private Pose[] parkingPositions;

    public static double intakeTimeOut = 0.8, reverseTime = 0.2;
    public static int reverseExtendoRetraction = 100;
    public static double outtakeWaitTime = 0.2;

    private final double[] cycleTime = {7,7,100,100};
    private final double[] goBackTime = {3.5,3.5,3.5,100};

    public static double parkTime = 1;

    public final DepositPixelDetector detector;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    public BlueNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
        drive.setRunMode(MecanumDrive.RunMode.PID);
        CoolIMU.imuOffset = 0;
        detector = new DepositPixelDetector(robot);
        timer.startTime();
        globalTimer.startTime();
        initPositions(detectionCase);
        initNodes(drive, robot);
    }

    private void initPositions(int detectionCase){
        switch (detectionCase){
            case 1:
                purplePosition = BluePositions.purplePosition1;
                preloadIntakeExtendoPosition = BluePositions.preloadIntakeExtendoPosition1;
                alignToCrossFieldForYellowPosition = BluePositions.alignToCrossFieldForYellowPosition1;
                crossFieldYellowPosition = BluePositions.crossFieldYellowPosition1;
                scoreYellowPosition = BluePositions.scoreYellowPosition1;
                scoreWhitePosition = BluePositions.scoreWhitePosition1;
                alignToCrossBackPosition = BluePositions.alignToCrossBackPosition1;
                intakePositions = BluePositions.intakePositions1;
                extendoDistanceTolerance = BluePositions.extendoDistanceTolerance1;
                extendoHeadingTolerance = BluePositions.extendoHeadingTolerance1;
                extendoPositions = BluePositions.extendoPositions1;
                scorePositions = BluePositions.scorePositions1;
                parkingPositions = BluePositions.parkingPositions1;
                startWaitTime = 4;
                break;
            case 2:
                purplePosition = BluePositions.purplePosition2;
                preloadIntakeExtendoPosition = BluePositions.preloadIntakeExtendoPosition2;
                alignToCrossFieldForYellowPosition = BluePositions.alignToCrossFieldForYellowPosition2;
                crossFieldYellowPosition = BluePositions.crossFieldYellowPosition2;
                scoreYellowPosition = BluePositions.scoreYellowPosition2;
                scoreWhitePosition = BluePositions.scoreWhitePosition2;
                alignToCrossBackPosition = BluePositions.alignToCrossBackPosition2;
                intakePositions = BluePositions.intakePositions2;
                extendoDistanceTolerance = BluePositions.extendoDistanceTolerance2;
                extendoHeadingTolerance = BluePositions.extendoHeadingTolerance2;
                extendoPositions = BluePositions.extendoPositions2;
                scorePositions = BluePositions.scorePositions2;
                parkingPositions = BluePositions.parkingPositions2;
                startWaitTime = 4;
                break;
            case 3:
                purplePosition = BluePositions.purplePosition3;
                preloadIntakeExtendoPosition = BluePositions.preloadIntakeExtendoPosition3;
                alignToCrossFieldForYellowPosition = BluePositions.alignToCrossFieldForYellowPosition3;
                crossFieldYellowPosition = BluePositions.crossFieldYellowPosition3;
                scoreYellowPosition = BluePositions.scoreYellowPosition3;
                scoreWhitePosition = BluePositions.scoreWhitePosition3;
                alignToCrossBackPosition = BluePositions.alignToCrossBackPosition3;
                intakePositions = BluePositions.intakePositions3;
                extendoDistanceTolerance = BluePositions.extendoDistanceTolerance3;
                extendoHeadingTolerance = BluePositions.extendoHeadingTolerance3;
                extendoPositions = BluePositions.extendoPositions3;
                scorePositions = BluePositions.scorePositions3;
                parkingPositions = BluePositions.parkingPositions3;
                startWaitTime = 4;
                break;
        }
    }

    public static int maxRetries = 3;

    private int intakeTries = 0;

    public LogicNode currentNode = new LogicNode("Start");

    private LogicNode scorePurple = new LogicNode("Scoring purple");
    private LogicNode waitForPurpleToFallIntakeStack = new LogicNode("Dropping purple while starting to intake");
    private LogicNode intake = new LogicNode("Intaking");
    private LogicNode reverseToRetry = new LogicNode("Reversing to retry intake");
    private LogicNode reverseToLeave = new LogicNode("Reversing to leave");
    private LogicNode alignToCross = new LogicNode("Aligning to cross");
    private LogicNode crossForYellow = new LogicNode("Crossing for yellow");
    private LogicNode scoreWhite = new LogicNode("Scoring white");
    private LogicNode waitWhite = new LogicNode("Waiting for white to fall");
    private LogicNode retryTransfer1 = new LogicNode("Extending intake to retry transfer");
    private LogicNode retryTransfer2 = new LogicNode("Retracting intake to retry transfer");
    private LogicNode scoreYellow = new LogicNode("Scoring yellow");
    private LogicNode waitYellow = new LogicNode("Waiting for yellow to fall");
    private LogicNode waitYellow2 = new LogicNode("Waiting for yellow to fall 2");
    private LogicNode alignToCrossBack = new LogicNode("Aligning to cross back");
    private LogicNode goToIntakePosition = new LogicNode("Going to intake position");
    private LogicNode goToScoringPosition = new LogicNode("Going to scoring position");
    private LogicNode waitForOuttake = new LogicNode("Waiting for outtake");
    private LogicNode waitToOpen = new LogicNode("Waiting to open");
    private LogicNode park = new LogicNode("Parking");
    private LogicNode startWait = new LogicNode("Waiting");

    public static double startWaitTime;

    public static int extendoRetryPosition = 100;

    private void initNodes(MecanumDrive drive, RobotModules robot){
        currentNode.addCondition(()->true, ()->{
            DropDown.index = 4;
//            Lift.profiled = true;
            Lift.level = 0;
            timer.reset();
            globalTimer.reset();
        }, startWait);

        startWait.addCondition(()->timer.seconds()>= startWaitTime, ()->{
            timer.reset();
            drive.setTargetPose(purplePosition);
            robot.outtake.setState(Outtake.State.GO_PURPLE);
        }, scorePurple);

        scorePurple.addCondition(()->
                (drive.reachedTarget(1) && drive.reachedHeading(0.2) && drive.stopped() && robot.outtake.getState() == Outtake.State.PURPLE),
                ()->{
                    robot.bottomGripper.setState(BottomGripper.State.OPENING);
                    Extendo.extendedPos = preloadIntakeExtendoPosition;
                    robot.intake.setState(Intake.State.START_INTAKE);
                    robot.extendo.setState(Extendo.State.GOING_LOCK);
                    timer.reset();
                }, waitForPurpleToFallIntakeStack);

        waitForPurpleToFallIntakeStack.addCondition(()->
                (timer.seconds() >= fallTime && robot.extendo.getState() == Extendo.State.LOCK), ()->{
            timer.reset();
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            intakeTries = 0;
                }, intake);

        intake.addCondition(()->detector.getPrevPixels() == 0 && detector.getPixels() == 1,
                ()->DropDown.index = Math.max(DropDown.index -1, 0), intake);

        intake.addCondition(()->detector.getPixels() == 2, ()-> {
            robot.intake.setState(Intake.State.REVERSE);
            Extendo.extendedPos = Extendo.extendedPos - reverseExtendoRetraction;
            robot.ramp.setState(Ramp.State.UP);
            timer.reset();
        }, reverseToLeave);

        intake.addCondition(()->detector.getPixels() == 1 && timer.seconds() >= intakeTimeOut && intakeTries<maxRetries, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            Extendo.extendedPos = Extendo.extendedPos - reverseExtendoRetraction;
            robot.ramp.setState(Ramp.State.UP);
            intakeTries++;
            timer.reset();
        }, reverseToRetry);

        intake.addCondition(()->(detector.getPixels() == 1 && timer.seconds() >= intakeTimeOut && intakeTries>=maxRetries) || ((30-parkTime) - globalTimer.seconds() <= (cycle<0?yellowCycleTime:goBackTime[cycle])), ()->{
            robot.intake.setState(Intake.State.REVERSE);
            Extendo.extendedPos = Extendo.extendedPos - reverseExtendoRetraction;
            robot.ramp.setState(Ramp.State.UP);
            timer.reset();
        }, reverseToLeave);

        intake.addCondition(()->detector.getPixels() == 0 && timer.seconds() >= intakeTimeOut, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            Extendo.extendedPos = Extendo.extendedPos - reverseExtendoRetraction;
            robot.ramp.setState(Ramp.State.UP);
            intakeTries++;
            timer.reset();
        }, reverseToRetry);

        reverseToRetry.addCondition(()->timer.seconds() >= reverseTime, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            DropDown.index = Math.max(0, DropDown.index - 1);
            Extendo.extendedPos = Extendo.extendedPos + reverseExtendoRetraction;
            robot.ramp.setState(Ramp.State.DOWN);
            timer.reset();
        }, intake);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == -1, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(alignToCrossFieldForYellowPosition);
            robot.ramp.setState(Ramp.State.DOWN);
        }, alignToCross);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle >= 0, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(scorePositions[cycle]);
            robot.ramp.setState(Ramp.State.DOWN);
        }, goToScoringPosition);

        alignToCross.addPositionCondition(drive, 2, crossFieldYellowPosition, crossForYellow);

        crossForYellow.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() >= outtakeActivationLine && robot.outtake.getState() == Outtake.State.DOWN &&
                robot.extendo.getState() == Extendo.State.IN && robot.topGripper.getState() == TopGripper.State.CLOSED &&
                robot.bottomGripper.getState() == BottomGripper.State.CLOSED, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_CLOSE);
                }, crossForYellow);

        crossForYellow.addCondition(()->robot.extendo.getState() == Extendo.State.IN && drive.reachedTarget(8), ()->{
            drive.setTargetPose(scoreWhitePosition);
        }, scoreWhite);

        scoreWhite.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        scoreWhite.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() >= outtakeActivationLine && robot.outtake.getState() == Outtake.State.DOWN &&
                robot.extendo.getState() == Extendo.State.IN && robot.topGripper.getState() == TopGripper.State.CLOSED &&
                robot.bottomGripper.getState() == BottomGripper.State.CLOSED, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_CLOSE);
        }, scoreWhite);

        scoreWhite.addCondition(()->drive.reachedTarget(1)/* && drive.stopped()*/ && robot.outtake.getState() == Outtake.State.UP, ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.extension.setState(Extension.State.BOOP);
            timer.reset();
        }, waitWhite);

        waitWhite.addCondition(()->timer.seconds() >= 0, ()->drive.setTargetPose(scoreYellowPosition), scoreYellow);

        scoreYellow.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && drive.stopped() && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            if(robot.bottomGripper.getState() == BottomGripper.State.CLOSED) robot.bottomGripper.setState(BottomGripper.State.OPENING);
            DropDown.index = 4;
        }, waitYellow);

        waitYellow.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            robot.extension.setState(Extension.State.GOING_IN);
            timer.reset();
        }, waitYellow2);

        waitYellow2.addCondition(()->timer.seconds() >= 0.1, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            Lift.level = 3;
//            Lift.profiled = false;
            DropDown.index = Math.max(DropDown.index -1, 0);
            drive.setTargetPose(alignToCrossBackPosition);
        }, alignToCrossBack);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1]))  && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            if(robot.bottomGripper.getState() == BottomGripper.State.CLOSED) robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        alignToCrossBack.addCondition(()->drive.reachedTarget(8), ()->{
            drive.setTargetPose(intakePositions[cycle+1]);
            cycle = 0;
        }, goToIntakePosition);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && detector.getPixels() != 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            if(robot.bottomGripper.getState() == BottomGripper.State.CLOSED) robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            Lift.level = 3;
            Extendo.extendedPos = extendoRetryPosition;
            robot.extendo.setState(Extendo.State.GOING_LOCK);
        }, retryTransfer1);

        retryTransfer1.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        retryTransfer1.addCondition(()->robot.extendo.getState() == Extendo.State.LOCK && robot.outtake.getState() == Outtake.State.DOWN,()->{
            robot.intake.setState(Intake.State.GOING_IN);
        }, retryTransfer2);

        retryTransfer2.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED && cycle == -1, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_CLOSE);
        }, scoreYellow);

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED && cycle >= 0, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
        }, waitForOuttake);

        waitForOuttake.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        waitForOuttake.addCondition(()->drive.reachedTarget(2) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
        }, waitToOpen);

        waitToOpen.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            drive.setTargetPose(intakePositions[cycle+1]);
            cycle++;
            DropDown.index = Math.max(DropDown.index -1, 0);
            if(cycle == 2) DropDown.index = 3;
        }, goToIntakePosition);

        waitForOuttake.addCondition(()->drive.reachedTarget(2) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            drive.setTargetPose(parkingPositions[cycle+1]);
            cycle++;
        }, park);

        waitForOuttake.addCondition(()->drive.reachedTarget(2) && detector.getPixels() != 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            Extendo.extendedPos = extendoRetryPosition;
            robot.extendo.setState(Extendo.State.GOING_LOCK);
        }, retryTransfer1);

        goToIntakePosition.addCondition(()->drive.reachedTarget(extendoDistanceTolerance[cycle]) && drive.reachedHeading(extendoHeadingTolerance[cycle]) && robot.extendo.getState() == Extendo.State.IN, ()->{
            Extendo.extendedPos = extendoPositions[cycle];
            robot.extendo.setState(Extendo.State.GOING_LOCK);
            robot.intake.setState(Intake.State.START_INTAKE);
        }, goToIntakePosition);

        goToIntakePosition.addCondition(()->robot.extendo.getState() == Extendo.State.LOCK, timer::reset, intake);

        goToScoringPosition.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() >= outtakeActivationLine && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED,
                ()->robot.outtake.setState(Outtake.State.GOING_UP_FAR), waitForOuttake);

    }

}
