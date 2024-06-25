package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.RedFarTrussPositions;
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
public class RedFarTrussNodes {

    private Pose purplePosition;
    private Pose preloadIntakePosition;

    private Pose alignToCrossFieldForYellowPosition;
    private Pose crossFieldYellowPosition;
    private Pose scoreYellowPosition;

    private final double yellowCycleTime = 8;

    public static double fallTime = 0.2;

    public int cycle = -1;

    private Pose[] alignToCrossPositions;
    private Pose[] alignToCrossBackPositions;
    private Pose[] crossBackPositions;
    private Pose[] intakePositions;
    public static double[] extendoDistanceTolerance;
    public static double[] extendoHeadingTolerance;
    private int[] extendoPositions;
    private Pose[] crossPositions;
    private Pose[] scorePositions;
    public static double outtakeActivationLine = -60;
    private Pose[] parkingPositions;

    public static double intakeTimeOut = 1.2, reverseTime = 0.3;
    public static int reverseExtendoRetraction = 1;
    public static double outtakeWaitTime = 0.1;
    public static double afterYellowDropTime = 0.2;

    public static double trussCrossPositionThresh = 3;
    public static double trussCrossHeadingThresh = 0.1;

    private final double[] cycleTime = {6,6,100,100};
    private final double[] goBackTime = {3,3,100,100};

    public static double headingOffset = -0.1;

    public static double parkTime = 1;

    public final DepositPixelDetector detector;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    public RedFarTrussNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
        drive.setRunMode(MecanumDrive.RunMode.PID);
        CoolIMU.imuOffset = 0;
        DropDown.index = 4;
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
                purplePosition = RedFarTrussPositions.purplePosition1;
                preloadIntakePosition = RedFarTrussPositions.preloadIntakePosition1;
                alignToCrossFieldForYellowPosition = RedFarTrussPositions.alignToCrossFieldForYellowPosition1;
                crossFieldYellowPosition = RedFarTrussPositions.crossFieldYellowPosition1;
                scoreYellowPosition = RedFarTrussPositions.scoreYellowPosition1;
                alignToCrossBackPositions = RedFarTrussPositions.alignToCrossBackPositions1;
                alignToCrossPositions = RedFarTrussPositions.alignToCrossPositions1;
                crossBackPositions = RedFarTrussPositions.crossBackPositions1;
                intakePositions = RedFarTrussPositions.intakePositions1;
                extendoDistanceTolerance = RedFarTrussPositions.extendoDistanceTolerance1;
                extendoHeadingTolerance = RedFarTrussPositions.extendoHeadingTolerance1;
                extendoPositions = RedFarTrussPositions.extendoPositions1;
                crossPositions = RedFarTrussPositions.crossPositions1;
                scorePositions = RedFarTrussPositions.scorePositions1;
                parkingPositions = RedFarTrussPositions.parkingPositions1;
                startWaitTime = 0;
                break;
            case 2:
                purplePosition = RedFarTrussPositions.purplePosition2;
                preloadIntakePosition = RedFarTrussPositions.preloadIntakePosition2;
                alignToCrossFieldForYellowPosition = RedFarTrussPositions.alignToCrossFieldForYellowPosition2;
                crossFieldYellowPosition = RedFarTrussPositions.crossFieldYellowPosition2;
                scoreYellowPosition = RedFarTrussPositions.scoreYellowPosition2;
                alignToCrossBackPositions = RedFarTrussPositions.alignToCrossBackPositions2;
                alignToCrossPositions = RedFarTrussPositions.alignToCrossPositions2;
                crossBackPositions = RedFarTrussPositions.crossBackPositions2;
                intakePositions = RedFarTrussPositions.intakePositions2;
                extendoDistanceTolerance = RedFarTrussPositions.extendoDistanceTolerance2;
                extendoHeadingTolerance = RedFarTrussPositions.extendoHeadingTolerance2;
                extendoPositions = RedFarTrussPositions.extendoPositions2;
                crossPositions = RedFarTrussPositions.crossPositions2;
                scorePositions = RedFarTrussPositions.scorePositions2;
                parkingPositions = RedFarTrussPositions.parkingPositions2;
                startWaitTime = 0;
                break;
            case 3:
                purplePosition = RedFarTrussPositions.purplePosition3;
                preloadIntakePosition = RedFarTrussPositions.preloadIntakePosition3;
                alignToCrossFieldForYellowPosition = RedFarTrussPositions.alignToCrossFieldForYellowPosition3;
                crossFieldYellowPosition = RedFarTrussPositions.crossFieldYellowPosition3;
                scoreYellowPosition = RedFarTrussPositions.scoreYellowPosition3;
                alignToCrossBackPositions = RedFarTrussPositions.alignToCrossBackPositions3;
                alignToCrossPositions = RedFarTrussPositions.alignToCrossPositions3;
                crossBackPositions = RedFarTrussPositions.crossBackPositions3;
                intakePositions = RedFarTrussPositions.intakePositions3;
                extendoDistanceTolerance = RedFarTrussPositions.extendoDistanceTolerance3;
                extendoHeadingTolerance = RedFarTrussPositions.extendoHeadingTolerance3;
                extendoPositions = RedFarTrussPositions.extendoPositions3;
                crossPositions = RedFarTrussPositions.crossPositions3;
                scorePositions = RedFarTrussPositions.scorePositions3;
                parkingPositions = RedFarTrussPositions.parkingPositions3;
                startWaitTime = 0;
                break;
        }
    }

    public static int maxRetries = 3;
    public static int headingCorrectionTries = 3;

    private int intakeTries = 0;

    public LogicNode currentNode = new LogicNode("Start");

    private LogicNode scorePurple = new LogicNode("Scoring purple");
    private LogicNode waitForPurpleToFall = new LogicNode("Dropping purple while starting to intake");
    private LogicNode goToIntakePositionAfterPurple = new LogicNode("Going to intake position after dropping purple");
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
    private LogicNode crossBack = new LogicNode("Crossing back");
    private LogicNode goToIntakePosition = new LogicNode("Going to intake position");
    private LogicNode crossField = new LogicNode("Crossing field");
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
//            Lift.profiled = true;
            Lift.level = 0;
        }, start);

        start.addCondition(()->timer.seconds()>=startWaitTime, ()->{
            if(detectionCase != 1) drive.setTargetPose(purplePosition);
            if(detectionCase == 1) drive.setTargetPose(RedFarTrussPositions.beforePurplePosition);
            robot.outtake.setState(Outtake.State.GO_PURPLE);
            timer.reset();
        }, scorePurple);

        scorePurple.addCondition(()->drive.reachedTarget(1) && drive.stopped() &&
                        drive.getTargetPose() == RedFarTrussPositions.beforePurplePosition && detectionCase == 1,
                ()->drive.setTargetPose(purplePosition), scorePurple);

        scorePurple.addCondition(()->
                        (drive.reachedTarget(1) && drive.reachedHeading(0.2) && drive.stopped() && robot.outtake.getState() == Outtake.State.PURPLE),
                ()->{
                    robot.bottomGripper.setState(BottomGripper.State.OPENING);
                    robot.intake.setState(Intake.State.START_INTAKE);
                    timer.reset();
                }, waitForPurpleToFall);

        waitForPurpleToFall.addCondition(()-> timer.seconds() >= fallTime, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(preloadIntakePosition);
        }, goToIntakePositionAfterPurple);

        goToIntakePositionAfterPurple.addCondition(()->drive.reachedTarget(1), ()->{
            timer.reset();
            intakeTries = 0;
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
            drive.setTargetPose(alignToCrossPositions[cycle]);
        }, alignToCross);

        alignToCross.addCondition(()->drive.reachedTarget(trussCrossPositionThresh) && drive.reachedHeading(trussCrossHeadingThresh)
                && cycle == -1 && robot.extendo.getState() == Extendo.State.IN && globalTimer.seconds() >= startWaitTime, ()->{
            drive.setTargetPose(crossFieldYellowPosition);
        }, crossForYellow);

        alignToCross.addCondition(()->drive.reachedTarget(trussCrossPositionThresh) && drive.reachedHeading(trussCrossHeadingThresh)
                && cycle >= 0 && robot.extendo.getState() == Extendo.State.IN, ()->{
            drive.setTargetPose(crossPositions[cycle]);
        }, crossField);

        crossField.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= outtakeActivationLine &&
                        robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED
                        && robot.outtake.getState() == Outtake.State.DOWN,
                ()->{
                    robot.activeIntake.setState(ActiveIntake.State.IDLE);
                    robot.outtake.setState(Outtake.State.GOING_UP_FAR);
                }, crossField);

        crossField.addPositionCondition(drive, 10, scorePositions[cycle+1], ()->{
            if(robot.outtake.getState() == Outtake.State.DOWN){
                robot.activeIntake.setState(ActiveIntake.State.IDLE);
                robot.outtake.setState(Outtake.State.GOING_UP_FAR);
            }
        }, waitForOuttake);

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
            if(robot.outtake.getState() == Outtake.State.DOWN) robot.outtake.setState(Outtake.State.GOING_UP_FAR);
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
            DropDown.index = 3;
        }, waitYellow);

        waitYellow.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            robot.extension.setState(Extension.State.GOING_MID);
            timer.reset();
        }, waitYellow2);

        waitYellow2.addCondition(()->timer.seconds() >= afterYellowDropTime, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            Lift.level = 3;
            Pivot.index = 1;
            drive.setTargetPose(alignToCrossBackPositions[cycle+1]);
        }, alignToCrossBack);

        scoreYellow.addCondition(()->drive.reachedTarget(2) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1]))  && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(parkingPositions[cycle+1]);
        }, park);

        alignToCrossBack.addCondition(()->drive.reachedTarget(trussCrossPositionThresh) && drive.reachedHeading(trussCrossHeadingThresh), ()->{
            drive.setTargetPose(crossBackPositions[cycle+1]);
        }, crossBack);

        crossBack.addCondition(()->drive.reachedTarget( 5),()->{
            drive.setTargetPose(intakePositions[cycle+1]);
            cycle++;
        } , goToIntakePosition);

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

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED && cycle == -1, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
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
            drive.setTargetPose(alignToCrossBackPositions[cycle+1]);
            DropDown.index = Math.max(DropDown.index -1, 0);
        }, alignToCrossBack);

//        align.addCondition(()->drive.reachedHeading(alignHeadingTolerance), ()-> {
//            drive.setTargetPose(intakePositions[cycle]);
//        }, goToIntakePosition);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle+1])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            drive.setTargetPose(parkingPositions[cycle+1]);
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
//            MecanumDrive.headingMultiplier = 3;
        }, intake);

    }

}
