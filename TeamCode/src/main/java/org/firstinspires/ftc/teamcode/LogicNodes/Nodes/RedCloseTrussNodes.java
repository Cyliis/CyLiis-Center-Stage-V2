package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.RedCloseTrussPositions;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.RedCloseTrussPositions;
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
public class RedCloseTrussNodes {

    private Pose purplePosition;

    private Pose alignToScoreYellowPosition;
    private Pose scoreYellowPosition;

    public static double fallTime = 0.2;

    public int cycle = 0;

    private Pose[] alignToCrossBackPositions;
    private Pose[] crossBackPositions;
    private Pose[] intakePositions;
    private Pose[] alignToCrossPositions;
    private Pose[] crossPositions;
    private Pose[] scorePositions;
    public static double[] extendoDistanceTolerance;
    public static double[] extendoHeadingTolerance;
    private int[] extendoPositions;
    public static double outtakeActivationLine = -60;
    private Pose[] parkingPositions;

    public static double intakeTimeOut = 1.2, reverseTime = 0.3;
    public static int reverseExtendoRetraction = 1;
    public static double outtakeWaitTime = 0.1;
    public static double afterYellowDropTime = 0.2;

    public static double trussCrossPositionThresh = 3;
    public static double trussCrossHeadingThresh = 0.1;

    private final double[] cycleTime = {6,6,6,100};
    private final double[] goBackTime = {3,3,3,100};

    public static double headingOffset = -0.1;

    public static double parkTime = 1;

    public final DepositPixelDetector detector;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    public RedCloseTrussNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
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
                purplePosition = RedCloseTrussPositions.purplePosition1;
                alignToScoreYellowPosition = RedCloseTrussPositions.alignToScoreYellowPosition1;
                scoreYellowPosition = RedCloseTrussPositions.scoreYellowPosition1;
                alignToCrossBackPositions = RedCloseTrussPositions.alignToCrossBackPositions1;
                crossBackPositions = RedCloseTrussPositions.crossBackPositions1;
                intakePositions = RedCloseTrussPositions.intakePositions1;
                alignToCrossPositions = RedCloseTrussPositions.alignToCrossPositions1;
                crossPositions = RedCloseTrussPositions.crossPositions1;
                scorePositions = RedCloseTrussPositions.scorePositions1;
                extendoDistanceTolerance = RedCloseTrussPositions.extendoDistanceTolerance1;
                extendoHeadingTolerance = RedCloseTrussPositions.extendoHeadingTolerance1;
                extendoPositions = RedCloseTrussPositions.extendoPositions1;
                parkingPositions = RedCloseTrussPositions.parkingPositions1;
                break;
            case 2:
                purplePosition = RedCloseTrussPositions.purplePosition2;
                alignToScoreYellowPosition = RedCloseTrussPositions.alignToScoreYellowPosition2;
                scoreYellowPosition = RedCloseTrussPositions.scoreYellowPosition2;
                alignToCrossBackPositions = RedCloseTrussPositions.alignToCrossBackPositions2;
                crossBackPositions = RedCloseTrussPositions.crossBackPositions2;
                intakePositions = RedCloseTrussPositions.intakePositions2;
                alignToCrossPositions = RedCloseTrussPositions.alignToCrossPositions2;
                crossPositions = RedCloseTrussPositions.crossPositions2;
                scorePositions = RedCloseTrussPositions.scorePositions2;
                extendoDistanceTolerance = RedCloseTrussPositions.extendoDistanceTolerance2;
                extendoHeadingTolerance = RedCloseTrussPositions.extendoHeadingTolerance2;
                extendoPositions = RedCloseTrussPositions.extendoPositions2;
                parkingPositions = RedCloseTrussPositions.parkingPositions2;
                break;
            case 3:
                purplePosition = RedCloseTrussPositions.purplePosition3;
                alignToScoreYellowPosition = RedCloseTrussPositions.alignToScoreYellowPosition3;
                scoreYellowPosition = RedCloseTrussPositions.scoreYellowPosition3;
                alignToCrossBackPositions = RedCloseTrussPositions.alignToCrossBackPositions3;
                crossBackPositions = RedCloseTrussPositions.crossBackPositions3;
                intakePositions = RedCloseTrussPositions.intakePositions3;
                alignToCrossPositions = RedCloseTrussPositions.alignToCrossPositions3;
                crossPositions = RedCloseTrussPositions.crossPositions3;
                scorePositions = RedCloseTrussPositions.scorePositions3;
                extendoDistanceTolerance = RedCloseTrussPositions.extendoDistanceTolerance3;
                extendoHeadingTolerance = RedCloseTrussPositions.extendoHeadingTolerance3;
                extendoPositions = RedCloseTrussPositions.extendoPositions3;
                parkingPositions = RedCloseTrussPositions.parkingPositions3;
                break;
        }
    }

    public static int maxRetries = 3;
    public static int headingCorrectionTries = 3;

    private int intakeTries = 0;

    public LogicNode currentNode = new LogicNode("Start");

    private LogicNode scorePurple = new LogicNode("Scoring purple");
    private LogicNode waitForPurpleToFall= new LogicNode("Dropping purple");
    private LogicNode alignToScoreYellow = new LogicNode("Aligning to score yellow");
    private LogicNode scoreYellow = new LogicNode("Scoring yellow");
    private LogicNode waitYellow = new LogicNode("Waiting for yellow to fall");
    private LogicNode waitYellow2 = new LogicNode("Waiting for yellow to fall 2");
    private LogicNode alignToCrossBack = new LogicNode("Aligning to cross back");
    private LogicNode crossBack = new LogicNode("Crossing back");
    private LogicNode goToIntakePosition = new LogicNode("Going to intake position");
    private LogicNode intake = new LogicNode("Intaking");
    private LogicNode reverseToRetry = new LogicNode("Reversing to retry intake");
    private LogicNode reverseToLeave = new LogicNode("Reversing to leave");
    private LogicNode alignToCross = new LogicNode("Aligning to cross");
    private LogicNode crossField = new LogicNode("Crossing field");
    private LogicNode waitForOuttake = new LogicNode("Waiting for outtake");
    private LogicNode waitToOpen = new LogicNode("Waiting to open");
    private LogicNode retryTransfer1 = new LogicNode("Extending intake to retry transfer");
    private LogicNode retryTransfer2 = new LogicNode("Retracting intake to retry transfer");
    private LogicNode park = new LogicNode("Parking");

    public static int extendoRetryPosition = 1;

    private void initNodes(MecanumDrive drive, RobotModules robot){
        currentNode.addCondition(()->true, ()->{
            timer.reset();
            globalTimer.reset();
            drive.setTargetPose(purplePosition);
            robot.outtake.setState(Outtake.State.GO_PURPLE);
            DropDown.index = 4;
//            Lift.profiled = true;
            Lift.level = 0;
        }, scorePurple);

        scorePurple.addCondition(()->
                        (drive.reachedTarget(1) && drive.reachedHeading(0.2) && drive.stopped() && robot.outtake.getState() == Outtake.State.PURPLE),
                ()->{
                    robot.bottomGripper.setState(BottomGripper.State.OPENING);
                    timer.reset();
                }, waitForPurpleToFall);

        waitForPurpleToFall.addCondition(()-> timer.seconds() >= fallTime, ()->{
            timer.reset();
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            intakeTries = 0;
            if(detectionCase  == 1) Pivot.index = 0;
            if(detectionCase == 2) Pivot.index = 3;
            if(detectionCase == 3) Pivot.index = 2;
            Lift.level = 0;
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
        }, alignToScoreYellow);

        alignToScoreYellow.addCondition(()->drive.reachedTarget(8), ()->{
            drive.setTargetPose(scoreYellowPosition);
        }, scoreYellow);

        scoreYellow.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle]);
        }, park);

        scoreYellow.addCondition(()->drive.reachedTarget(1) && drive.stopped() && robot.outtake.getState() == Outtake.State.UP && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
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
            drive.setTargetPose(alignToCrossBackPositions[cycle]);
        }, alignToCrossBack);

        alignToCrossBack.addCondition(()->drive.reachedTarget(trussCrossPositionThresh) && drive.reachedHeading(trussCrossHeadingThresh), ()->{
            drive.setTargetPose(crossBackPositions[cycle]);
        }, crossBack);

        crossBack.addCondition(()->drive.reachedTarget( 5),()->{
            drive.setTargetPose(intakePositions[cycle]);
        } , goToIntakePosition);

        goToIntakePosition.addCondition(()->drive.reachedTarget(extendoDistanceTolerance[cycle]) && drive.reachedHeading(extendoHeadingTolerance[cycle]) && robot.extendo.getState() == Extendo.State.IN, ()->{
            Extendo.extendedPos = extendoPositions[cycle];
            robot.extendo.setState(Extendo.State.GOING_LOCK);
            robot.intake.setState(Intake.State.START_INTAKE);
        }, goToIntakePosition);

        //            MecanumDrive.headingMultiplier = 3;
        goToIntakePosition.addCondition(()->robot.extendo.getState() == Extendo.State.LOCK, timer::reset, intake);

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

        intake.addCondition(()->(detector.getPixels() == 1 && ((timer.seconds() >= intakeTimeOut && intakeTries>=maxRetries) || cycle == 2)) ||
                ((30-parkTime) - globalTimer.seconds() <= goBackTime[cycle]), ()->{
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
                drive.setTargetPose(drive.getTargetPose().plus(new Pose(0,0,headingOffset)));
            }
            timer.reset();
        }, intake);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            robot.activeIntake.setState(ActiveIntake.State.HOLD);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(alignToCrossPositions[cycle]);
        }, alignToCross);

        alignToCross.addCondition(()->drive.reachedTarget(trussCrossPositionThresh) && drive.reachedHeading(trussCrossHeadingThresh)
                && robot.extendo.getState() == Extendo.State.IN, ()->{
            drive.setTargetPose(crossPositions[cycle]);
        }, crossField);

        crossField.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= outtakeActivationLine &&
                        robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED
                        && robot.outtake.getState() == Outtake.State.DOWN,
                ()->{
                    robot.activeIntake.setState(ActiveIntake.State.IDLE);
                    robot.outtake.setState(Outtake.State.GOING_UP_FAR);
                }, crossField);

        crossField.addPositionCondition(drive, 10, scorePositions[cycle], ()->{
            if(robot.outtake.getState() == Outtake.State.DOWN){
                robot.activeIntake.setState(ActiveIntake.State.IDLE);
                robot.outtake.setState(Outtake.State.GOING_UP_FAR);
            }
        }, waitForOuttake);

        retryTransfer1.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle]);
        }, park);

        retryTransfer1.addCondition(()->robot.extendo.getState() == Extendo.State.LOCK && robot.outtake.getState() == Outtake.State.DOWN,()->{
            robot.intake.setState(Intake.State.GOING_IN);
        }, retryTransfer2);

        retryTransfer2.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle]);
        }, park);

        retryTransfer2.addCondition(()->robot.extendo.getState() == Extendo.State.IN && robot.bottomGripper.getState() == BottomGripper.State.CLOSED && robot.topGripper.getState() == TopGripper.State.CLOSED, ()->{
            robot.outtake.setState(Outtake.State.GOING_UP_FAR);
        }, waitForOuttake);

        waitForOuttake.addCondition(()->globalTimer.seconds() >= (30 - parkTime), ()->{
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(parkingPositions[cycle]);
        }, park);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() >= (cycleTime[cycle])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
        }, waitToOpen);

        waitToOpen.addCondition(()->robot.topGripper.getState() == TopGripper.State.OPEN && robot.bottomGripper.getState() == BottomGripper.State.OPEN, ()->{
            cycle++;
            drive.setTargetPose(alignToCrossBackPositions[cycle]);
            DropDown.index = Math.max(DropDown.index -1, 0);
        }, alignToCrossBack);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() == 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime && ((30-parkTime) - globalTimer.seconds() < (cycleTime[cycle])), ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_DELAY);
            drive.setTargetPose(parkingPositions[cycle]);
        }, park);

        waitForOuttake.addCondition(()->drive.reachedTarget(4) && detector.getPixels() != 0 && robot.outtake.getState() == Outtake.State.UP && robot.outtake.timer.seconds() >= outtakeWaitTime, ()->{
            robot.topGripper.setState(TopGripper.State.OPENING);
            robot.bottomGripper.setState(BottomGripper.State.OPENING);
            robot.outtake.setState(Outtake.State.GOING_DOWN_SAFE);
            Extendo.extendedPos = extendoRetryPosition;
            robot.extendo.setState(Extendo.State.GOING_LOCK);
        }, retryTransfer1);

    }

}
