package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.RedPositions;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
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
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Red;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;

@Config
public class RedNodes {

    private Pose purplePosition;
    private int preloadIntakeExtendoPosition;

    private Pose alignToCrossFieldForYellowPosition;
    private Pose crossFieldYellowPosition;
    private Pose scoreYellowPosition;
    private Pose scoreWhitePosition;

    private final double yellowCycleTime = 8;

    public static double fallTime = 0.2;

    public int cycle = -1;

    private Pose alignToCrossBackPosition;
    private Pose[] intakePositions;
    public static double[] extendoDistanceTolerance;
    public static double[] extendoHeadingTolerance;
    private int[] extendoPositions;
    private Pose[] scorePositions;
    public static double outtakeActivationLine = -55;
    private Pose[] parkingPositions;

    public static double intakeTimeOut = 1, reverseTime = 0.2;
    public static int reverseExtendoRetraction = 100;
    public static double outtakeWaitTime = 0.1;

    private final double[] cycleTime = {6,6,6,100};
    private final double[] goBackTime = {3,3,3,100};

    public static double headingOffset = 0.1;

    public static double parkTime = 1;

    public final DepositPixelDetector detector;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    public RedNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
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
                purplePosition = RedPositions.purplePosition1;
                preloadIntakeExtendoPosition = RedPositions.preloadIntakeExtendoPosition1;
                alignToCrossFieldForYellowPosition = RedPositions.alignToCrossFieldForYellowPosition1;
                crossFieldYellowPosition = RedPositions.crossFieldYellowPosition1;
                scoreYellowPosition = RedPositions.scoreYellowPosition1;
                scoreWhitePosition = RedPositions.scoreWhitePosition1;
                alignToCrossBackPosition = RedPositions.alignToCrossBackPosition1;
                intakePositions = RedPositions.intakePositions1;
                extendoDistanceTolerance = RedPositions.extendoDistanceTolerance1;
                extendoHeadingTolerance = RedPositions.extendoHeadingTolerance1;
                extendoPositions = RedPositions.extendoPositions1;
                scorePositions = RedPositions.scorePositions1;
                parkingPositions = RedPositions.parkingPositions1;
                startWaitTime = 0;
                break;
            case 2:
                purplePosition = RedPositions.purplePosition2;
                preloadIntakeExtendoPosition = RedPositions.preloadIntakeExtendoPosition2;
                alignToCrossFieldForYellowPosition = RedPositions.alignToCrossFieldForYellowPosition2;
                crossFieldYellowPosition = RedPositions.crossFieldYellowPosition2;
                scoreYellowPosition = RedPositions.scoreYellowPosition2;
                scoreWhitePosition = RedPositions.scoreWhitePosition2;
                alignToCrossBackPosition = RedPositions.alignToCrossBackPosition2;
                intakePositions = RedPositions.intakePositions2;
                extendoDistanceTolerance = RedPositions.extendoDistanceTolerance2;
                extendoHeadingTolerance = RedPositions.extendoHeadingTolerance2;
                extendoPositions = RedPositions.extendoPositions2;
                scorePositions = RedPositions.scorePositions2;
                parkingPositions = RedPositions.parkingPositions2;
//                startWaitTime = 0.5;
                startWaitTime = 0;
                break;
            case 3:
                purplePosition = RedPositions.purplePosition3;
                preloadIntakeExtendoPosition = RedPositions.preloadIntakeExtendoPosition3;
                alignToCrossFieldForYellowPosition = RedPositions.alignToCrossFieldForYellowPosition3;
                crossFieldYellowPosition = RedPositions.crossFieldYellowPosition3;
                scoreYellowPosition = RedPositions.scoreYellowPosition3;
                scoreWhitePosition = RedPositions.scoreWhitePosition3;
                alignToCrossBackPosition = RedPositions.alignToCrossBackPosition3;
                intakePositions = RedPositions.intakePositions3;
                extendoDistanceTolerance = RedPositions.extendoDistanceTolerance3;
                extendoHeadingTolerance = RedPositions.extendoHeadingTolerance3;
                extendoPositions = RedPositions.extendoPositions3;
                scorePositions = RedPositions.scorePositions3;
                parkingPositions = RedPositions.parkingPositions3;
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
    private LogicNode align = new LogicNode("Ideea lui Codrin");
    private LogicNode park = new LogicNode("Parking");
    private LogicNode startWait = new LogicNode("Waiting");

    public static double startWaitTime;

    public static int extendoRetryPosition = 150;

    private void initNodes(MecanumDrive drive, RobotModules robot){
        currentNode.addCondition(()->true, ()->{
            timer.reset();
            globalTimer.reset();
            DropDown.index = 4;
//            Lift.profiled = true;
            Lift.level = 0;
        }, startWait);

        startWait.addCondition(()->timer.seconds()>=startWaitTime, ()->{
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
            MecanumDrive.headingMultiplier = 1;
        }, intake);

        intake.addCondition(()->detector.getPrevPixels() == 0 && detector.getPixels() == 1,
                ()->DropDown.index = Math.max(DropDown.index -1, 0), intake);

        intake.addCondition(()->detector.getPixels() == 2, ()-> {
            robot.intake.setState(Intake.State.REVERSE);
            MecanumDrive.headingMultiplier = 1;
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
            MecanumDrive.headingMultiplier = 1;
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
            if(intakeTries%headingCorrectionTries == 1 && intakeTries != 1) {
                drive.setTargetPose(drive.getTargetPose().plus(new Pose(0,0,headingOffset * (cycle>1?-1:1))));
            }
            timer.reset();
        }, intake);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == -1, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.activeIntake.setState(ActiveIntake.State.HOLD);
            robot.intake.setState(Intake.State.GOING_IN);
            robot.ramp.setState(Ramp.State.UP);
            drive.setTargetPose(alignToCrossFieldForYellowPosition);
        }, alignToCross);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle >= 0, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.activeIntake.setState(ActiveIntake.State.HOLD);
            robot.intake.setState(Intake.State.GOING_IN);
            robot.ramp.setState(Ramp.State.UP);
            drive.setTargetPose(scorePositions[cycle]);
        }, goToScoringPosition);

        alignToCross.addPositionCondition(drive, 2, crossFieldYellowPosition, crossForYellow);

        crossForYellow.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= outtakeActivationLine && robot.outtake.getState() == Outtake.State.DOWN &&
                robot.extendo.getState() == Extendo.State.IN && robot.topGripper.getState() == TopGripper.State.CLOSED &&
                robot.bottomGripper.getState() == BottomGripper.State.CLOSED, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_CLOSE);
        }, crossForYellow);

        crossForYellow.addCondition(()->robot.extendo.getState() == Extendo.State.IN && drive.reachedTarget(8), ()->{
            drive.setTargetPose(scoreWhitePosition);
            robot.activeIntake.setState(ActiveIntake.State.IDLE);
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

        scoreWhite.addCondition(()->drive.reachedTarget(2)/* && drive.stopped()*/ && robot.outtake.getState() == Outtake.State.UP, ()->{
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
//            if(Lift.level == 0) Lift.level = -1;
            DropDown.index = 4;
        }, waitYellow);

        waitYellow.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            robot.extension.setState(Extension.State.GOING_IN);
            timer.reset();
        }, waitYellow2);

        waitYellow2.addCondition(()->timer.seconds() >= 0.3, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            Lift.level = 3;
//            Lift.profiled = false;
            DropDown.index = Math.max(DropDown.index -1, 0);
            drive.setTargetPose(alignToCrossBackPosition);
        }, alignToCrossBack);

        scoreYellow.addCondition(()->drive.reachedTarget(2) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1]))  && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            if(robot.bottomGripper.getState() == BottomGripper.State.CLOSED) robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        alignToCrossBack.addCondition(()->drive.reachedTarget(8), ()->{
            drive.setTargetPose(intakePositions[cycle+1]);
            robot.ramp.setState(Ramp.State.DOWN);
            cycle = 0;
        }, goToIntakePosition);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && detector.getPixels() != 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            if(robot.bottomGripper.getState() == BottomGripper.State.CLOSED) robot.bottomGripper.setState(BottomGripper.State.OPENING);
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

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED && cycle == -1, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_CLOSE);
            Lift.level = 3;
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

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
        }, waitToOpen);

        waitToOpen.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            drive.setTargetPose(intakePositions[cycle+1]);
            cycle++;
            DropDown.index = Math.max(DropDown.index -1, 0);
            if(cycle == 2) DropDown.index = 2;
            robot.ramp.setState(Ramp.State.DOWN);
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

        goToIntakePosition.addCondition(()->drive.reachedTarget(extendoDistanceTolerance[cycle]) && drive.reachedHeading(extendoHeadingTolerance[cycle]) && robot.extendo.getState() == Extendo.State.IN, ()->{
            Extendo.extendedPos = extendoPositions[cycle];
            robot.extendo.setState(Extendo.State.GOING_LOCK);
            robot.intake.setState(Intake.State.START_INTAKE);
        }, goToIntakePosition);

        goToIntakePosition.addCondition(()->robot.extendo.getState() == Extendo.State.LOCK, ()->{
            timer.reset();
            MecanumDrive.headingMultiplier = 3;
        }, intake);

        goToScoringPosition.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= outtakeActivationLine && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED,
                ()->{
            robot.activeIntake.setState(ActiveIntake.State.IDLE);
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
                }, waitForOuttake);

    }

}
