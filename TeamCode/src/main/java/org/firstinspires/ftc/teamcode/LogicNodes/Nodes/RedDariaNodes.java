package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.RedDariaPositions;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.DepositPixelDetector;
import org.firstinspires.ftc.teamcode.Modules.Other.TopGripper;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pivot;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;

@Config
public class RedDariaNodes {

    private Pose purplePosition;
    private int preloadIntakeExtendoPosition;

    private Pose alignToCrossFieldForYellowPosition;
    private Pose crossFieldYellowPosition;
    private Pose scoreYellowPosition;

    private final double yellowCycleTime = 8;

    public static double fallTime = 0.2;

    public int cycle = -1;

    private Pose alignToCrossBackPosition;
    private Pose[] beforeIntakePositions;
    private Pose[] intakePositions;
    public static double[] extendoDistanceTolerance;
    public static double[] extendoHeadingTolerance;
    private int[] extendoPositions;
    private Pose[] beforeScoringPositions;
    private Pose[] scorePositions;
    public static double outtakeActivationLine = -55;
    private Pose[] parkingPositions;

    public static double intakeTimeOut = 1.2, reverseTime = 0.4;
    public static int reverseExtendoRetraction = 100;
    public static double outtakeWaitTime = 0.1;
    public static double afterYellowDropTime = 0.2;

    private final double[] cycleTime = {6,6,6,100};
    private final double[] goBackTime = {3,3,3,100};

    public static double headingOffset = 0.1;

    public static double parkTime = 1;

    public static double grabberClosingDelay = 0.2;

    public final DepositPixelDetector detector;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    public RedDariaNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
        drive.setRunMode(MecanumDrive.RunMode.PID);
        CoolIMU.imuOffset = 0;
        detector = new DepositPixelDetector(robot);
        timer.startTime();
        globalTimer.startTime();
        initPositions(detectionCase);
        initNodes(drive, robot);
    }

    public int detectionCase;

    private void initPositions(int detectionCase){
        this.detectionCase = detectionCase;
        switch (detectionCase){
            case 1:
                purplePosition = RedDariaPositions.purplePosition1;
                preloadIntakeExtendoPosition = RedDariaPositions.preloadIntakeExtendoPosition1;
                alignToCrossFieldForYellowPosition = RedDariaPositions.alignToCrossFieldForYellowPosition1;
                crossFieldYellowPosition = RedDariaPositions.crossFieldYellowPosition1;
                scoreYellowPosition = RedDariaPositions.scoreYellowPosition1;
                alignToCrossBackPosition = RedDariaPositions.alignToCrossBackPosition1;
                beforeIntakePositions = RedDariaPositions.beforeIntakePositions1;
                intakePositions = RedDariaPositions.intakePositions1;
                extendoDistanceTolerance = RedDariaPositions.extendoDistanceTolerance1;
                extendoHeadingTolerance = RedDariaPositions.extendoHeadingTolerance1;
                extendoPositions = RedDariaPositions.extendoPositions1;
                beforeScoringPositions = RedDariaPositions.beforeScoringPositions1;
                scorePositions = RedDariaPositions.scorePositions1;
                parkingPositions = RedDariaPositions.parkingPositions1;
                startWaitTime = 0;
                break;
            case 2:
                purplePosition = RedDariaPositions.purplePosition2;
                preloadIntakeExtendoPosition = RedDariaPositions.preloadIntakeExtendoPosition2;
                alignToCrossFieldForYellowPosition = RedDariaPositions.alignToCrossFieldForYellowPosition2;
                crossFieldYellowPosition = RedDariaPositions.crossFieldYellowPosition2;
                scoreYellowPosition = RedDariaPositions.scoreYellowPosition2;
                alignToCrossBackPosition = RedDariaPositions.alignToCrossBackPosition2;
                beforeIntakePositions = RedDariaPositions.beforeIntakePositions2;
                intakePositions = RedDariaPositions.intakePositions2;
                extendoDistanceTolerance = RedDariaPositions.extendoDistanceTolerance2;
                extendoHeadingTolerance = RedDariaPositions.extendoHeadingTolerance2;
                extendoPositions = RedDariaPositions.extendoPositions2;
                beforeScoringPositions = RedDariaPositions.beforeScoringPositions2;
                scorePositions = RedDariaPositions.scorePositions2;
                parkingPositions = RedDariaPositions.parkingPositions2;
                startWaitTime = 0;
                break;
            case 3:
                purplePosition = RedDariaPositions.purplePosition3;
                preloadIntakeExtendoPosition = RedDariaPositions.preloadIntakeExtendoPosition3;
                alignToCrossFieldForYellowPosition = RedDariaPositions.alignToCrossFieldForYellowPosition3;
                crossFieldYellowPosition = RedDariaPositions.crossFieldYellowPosition3;
                scoreYellowPosition = RedDariaPositions.scoreYellowPosition3;
                alignToCrossBackPosition = RedDariaPositions.alignToCrossBackPosition3;
                beforeIntakePositions = RedDariaPositions.beforeIntakePositions3;
                intakePositions = RedDariaPositions.intakePositions3;
                extendoDistanceTolerance = RedDariaPositions.extendoDistanceTolerance3;
                extendoHeadingTolerance = RedDariaPositions.extendoHeadingTolerance3;
                extendoPositions = RedDariaPositions.extendoPositions3;
                beforeScoringPositions = RedDariaPositions.beforeScoringPositions3;
                scorePositions = RedDariaPositions.scorePositions3;
                parkingPositions = RedDariaPositions.parkingPositions3;
                startWaitTime = 0;
                break;
        }
    }

    public static int maxRetries = 3;
    public static int headingCorrectionTries = 3;

    private int intakeTries = 0;

    public LogicNode currentNode = new LogicNode("Start");

    private LogicNode scorePurple = new LogicNode("Scoring purple");
    private LogicNode waitForPurpleToFallIntakeStack = new LogicNode("Dropping purple while starting to intake");
    private LogicNode intake = new LogicNode("Intaking");
    private LogicNode reverseToRetry = new LogicNode("Reversing to retry intake");
    private LogicNode reverseToLeave = new LogicNode("Reversing to leave");
    private LogicNode alignToCross = new LogicNode("Aligning to cross");
    private LogicNode crossForYellow = new LogicNode("Crossing for yellow");
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
    private LogicNode start = new LogicNode("Waiting");

    public static double startWaitTime;

    public static int extendoRetryPosition = 150;

    private void initNodes(MecanumDrive drive, RobotModules robot){
        currentNode.addCondition(()->true, ()->{
            timer.reset();
            globalTimer.reset();
            DropDown.index = 4;
            robot.dropDown.setState(DropDown.State.UP);
//            Lift.profiled = true;
            Lift.level = 0;
        }, start);

        start.addCondition(()->true, ()->{
            drive.setTargetPose(purplePosition);
            robot.outtake.setState(Outtake.State.GO_PURPLE);
            timer.reset();
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
//            MecanumDrive.headingMultiplier = 1;
        }, intake);

        intake.addCondition(()->detector.getPrevPixels() == 0 && detector.getPixels() == 1,
                ()->DropDown.index = Math.max(DropDown.index -1, 0), intake);

        intake.addCondition(()->detector.getPixels() == 2, ()-> {
            robot.intake.setState(Intake.State.REVERSE_DELAY);
//            MecanumDrive.headingMultiplier = 1;
            timer.reset();
        }, reverseToLeave);

        intake.addCondition(()->detector.getPixels() == 1 && timer.seconds() >= intakeTimeOut && intakeTries<maxRetries, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            Extendo.extendedPos = Extendo.extendedPos - reverseExtendoRetraction;
            intakeTries++;
            timer.reset();
        }, reverseToRetry);

        intake.addCondition(()->(detector.getPixels() == 1 && timer.seconds() >= intakeTimeOut && intakeTries>=maxRetries) || ((30-parkTime) - globalTimer.seconds() <= (cycle<0?yellowCycleTime:goBackTime[cycle])), ()->{
            robot.intake.setState(Intake.State.REVERSE);
//            MecanumDrive.headingMultiplier = 1;
            timer.reset();
        }, reverseToLeave);

        intake.addCondition(()->detector.getPixels() == 0 && timer.seconds() >= intakeTimeOut, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            Extendo.extendedPos = Extendo.extendedPos - reverseExtendoRetraction;
            intakeTries++;
            timer.reset();
        }, reverseToRetry);

        reverseToRetry.addCondition(()->timer.seconds() >= reverseTime, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            DropDown.index = Math.max(0, DropDown.index - 1);
            Extendo.extendedPos = Extendo.extendedPos + reverseExtendoRetraction;
            if(intakeTries%headingCorrectionTries == 1 && intakeTries != 1) {
                drive.setTargetPose(drive.getTargetPose().plus(new Pose(0,0,headingOffset * (cycle>1?-1:1))));
            }
            timer.reset();
        }, intake);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == -1 && robot.outtake.getState() == Outtake.State.DOWN, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.activeIntake.setState(ActiveIntake.State.REVERSE_SAFETY);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(alignToCrossFieldForYellowPosition);
        }, alignToCross);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle >= 0, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.activeIntake.setState(ActiveIntake.State.REVERSE_SAFETY);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(beforeScoringPositions[cycle]);
        }, goToScoringPosition);

        alignToCross.addCondition(()->drive.reachedTarget(2) && globalTimer.seconds() >= startWaitTime,
                ()->drive.setTargetPose(crossFieldYellowPosition), crossForYellow);

        crossForYellow.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= outtakeActivationLine && robot.outtake.getState() == Outtake.State.DOWN &&
                robot.extendo.getState() == Extendo.State.IN && robot.topGripper.getState() == TopGripper.State.CLOSED &&
                robot.bottomGripper.getState() == BottomGripper.State.CLOSED, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
        }, crossForYellow);

        crossForYellow.addCondition(()->robot.extendo.getState() == Extendo.State.IN && drive.reachedTarget(8), ()->{
            drive.setTargetPose(scoreYellowPosition);
            robot.activeIntake.setState(ActiveIntake.State.IDLE);
            if(detectionCase  == 1) Pivot.index = 0;
            if(detectionCase == 2) Pivot.index = 3;
            if(detectionCase == 3) Pivot.index = 2;
            Lift.level = 0;
        }, scoreYellow);

        scoreYellow.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && drive.stopped() && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
//            if(Lift.level == 0) Lift.level = -1;
            DropDown.index = 4;
        }, waitYellow);

        waitYellow.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            robot.extension.setState(Extension.State.GOING_MID);
            timer.reset();
        }, waitYellow2);

        waitYellow2.addCondition(()->timer.seconds() >= afterYellowDropTime, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            Lift.level = 3;
            Pivot.index = 1;
            DropDown.index = Math.max(DropDown.index -1, 0);
            drive.setTargetPose(alignToCrossBackPosition);
        }, alignToCrossBack);

        scoreYellow.addCondition(()->drive.reachedTarget(2) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1]))  && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        alignToCrossBack.addCondition(()->drive.reachedTarget(8), ()->{
            drive.setTargetPose(intakePositions[cycle+1]);
            cycle = 0;
        }, goToIntakePosition);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && detector.getPixels() != 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_SAFE);
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

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED &&
                robot.topGripper.getState() == TopGripper.State.CLOSED && cycle == -1  && robot.bottomGripper.timer.seconds() >= grabberClosingDelay, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
            Lift.level = 3;
        }, scoreYellow);

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED
                && robot.topGripper.getState() == TopGripper.State.CLOSED && cycle >= 0  && robot.bottomGripper.timer.seconds() >= grabberClosingDelay, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
        }, waitForOuttake);

        waitForOuttake.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
        }, waitToOpen);

        waitToOpen.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            drive.setTargetPose(beforeIntakePositions[cycle+1]);
            cycle++;
            DropDown.index = Math.max(DropDown.index -1, 0);
            if(cycle == 2) DropDown.index = 2;
        }, goToIntakePosition);

//        align.addCondition(()->drive.reachedHeading(alignHeadingTolerance), ()-> {
//            drive.setTargetPose(intakePositions[cycle]);
//        }, goToIntakePosition);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            drive.setTargetPose(parkingPositions[cycle+1]);
            cycle++;
        }, park);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() != 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_SAFE);
            Extendo.extendedPos = extendoRetryPosition;
            robot.extendo.setState(Extendo.State.GOING_LOCK);
        }, retryTransfer1);

        goToIntakePosition.addCondition(()->cycle!=0 && drive.reachedTarget(10) && drive.reachedHeading(0.1) &&
                drive.getTargetPose() == beforeIntakePositions[cycle], ()->drive.setTargetPose(intakePositions[cycle]), goToIntakePosition);

        goToIntakePosition.addCondition(()->drive.reachedTarget(extendoDistanceTolerance[cycle]) && drive.reachedHeading(extendoHeadingTolerance[cycle])
                && robot.extendo.getState() == Extendo.State.IN && drive.getTargetPose() == intakePositions[cycle], ()->{
            Extendo.extendedPos = extendoPositions[cycle];
            robot.extendo.setState(Extendo.State.GOING_LOCK);
            robot.intake.setState(Intake.State.START_INTAKE);
        }, goToIntakePosition);

        goToIntakePosition.addCondition(()->robot.extendo.getState() == Extendo.State.LOCK, ()->{
            timer.reset();
//            MecanumDrive.headingMultiplier = 3;
        }, intake);

        goToScoringPosition.addCondition(()->drive.getTargetPose() == beforeScoringPositions[cycle] && drive.reachedTarget(10),
                ()->drive.setTargetPose(scorePositions[cycle]), goToScoringPosition);

        goToScoringPosition.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= outtakeActivationLine && robot.bottomGripper.getState() == BottomGripper.State.CLOSED
                        && robot.topGripper.getState() == TopGripper.State.CLOSED && robot.bottomGripper.timer.seconds() >= grabberClosingDelay &&
                        drive.getTargetPose() == scorePositions[cycle],
                ()->{
                    robot.activeIntake.setState(ActiveIntake.State.IDLE);
                    robot.outtake.setState(Outtake.State.GOING_UP_FAR);
                }, waitForOuttake);

    }

}
