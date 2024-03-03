package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.LogicNodes.Positions.BluePositions;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Other.BottomGripper;
import org.firstinspires.ftc.teamcode.Modules.Other.DepositPixelDetector;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;

@Config
public class BlueNodes {

    private Pose purplePosition;
    public static final double purpleFallTime = 0.2;
    private int preloadIntakeExtendoPosition;

    private Pose alignToCrossFieldForYellowPosition;
    private Pose crossFieldYellowPosition;
    private Pose scoreYellowPosition;
    private Pose scoreWhitePosition;

    public static double fallTime = 0.2;

    public int cycle = -1;

    private Pose alignToCrossBackPosition;
    private Pose[] intakePositions;
    private int[] extendoPositions;
    private Pose[] scorePositions;
    public static double outtakeActivationLine = 70;
    private Pose[] parkingPositions;

    public static double intakeTimeOut = 1, reverseTime = 0.2;

    private double[] cycleTime = {6,6,6.5,6.5};
    private double[] goBackTime = {2.5,2.5,3,3};

    public static double parkTime = 0.5;

    public final DepositPixelDetector detector;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    public BlueNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
        drive.setRunMode(MecanumDrive.RunMode.PID);
        CoolIMU.imuOffset = 0;
        robot.bottomGripper.setState(BottomGripper.State.CLOSING);
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
                extendoPositions = BluePositions.extendoPositions1;
                scorePositions = BluePositions.scorePositions1;
                parkingPositions = BluePositions.parkingPositions1;
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
                extendoPositions = BluePositions.extendoPositions2;
                scorePositions = BluePositions.scorePositions2;
                parkingPositions = BluePositions.parkingPositions2;
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
                extendoPositions = BluePositions.extendoPositions3;
                scorePositions = BluePositions.scorePositions3;
                parkingPositions = BluePositions.parkingPositions3;
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
    private LogicNode retryTransfer1 = new LogicNode("Extending intake to retry transfer");
    private LogicNode retryTransfer2 = new LogicNode("Retracting intake to retry transfer");
    private LogicNode scoreYellow = new LogicNode("Scoring yellow");
    private LogicNode alignToCrossBack = new LogicNode("Aligning to cross back");
    private LogicNode goToIntakePosition = new LogicNode("Going to intake position");
    private LogicNode goToScoringPosition = new LogicNode("Going to scoring position");
    private LogicNode waitForOuttake = new LogicNode("Waiting for outtake");

    private void initNodes(MecanumDrive drive, RobotModules robot){
        currentNode.addCondition(()->true, ()->{
            timer.reset();
            globalTimer.reset();
            drive.setTargetPose(purplePosition);
            robot.outtake.setState(Outtake.State.GO_PURPLE);
        }, scorePurple);

        scorePurple.addCondition(()->
                (drive.reachedTarget(1) && drive.reachedHeading(0.2) && robot.outtake.getState() == Outtake.State.PURPLE),
                ()->{
                    robot.bottomGripper.setState(BottomGripper.State.OPENING);
                    Extendo.extendedPos = preloadIntakeExtendoPosition;
                    robot.intake.setState(Intake.State.START_INTAKE);
                    robot.extendo.setState(Extendo.State.GOING_OUT);
                    timer.reset();
                }, waitForPurpleToFallIntakeStack);

        waitForPurpleToFallIntakeStack.addCondition(()->
                (timer.seconds() >= fallTime && robot.extendo.getState() == Extendo.State.OUT), ()->{
            timer.reset();
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            intakeTries = 0;
                }, intake);

        intake.addCondition(()->detector.getPixels() == 2, ()-> {
            robot.intake.setState(Intake.State.REVERSE);
            timer.reset();
        }, reverseToLeave);

        intake.addCondition(()->detector.getPixels() == 1 && timer.seconds() >= intakeTimeOut && intakeTries<maxRetries, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            intakeTries++;
            timer.reset();
        }, reverseToRetry);

        intake.addCondition(()->detector.getPixels() == 1 && timer.seconds() >= intakeTimeOut && intakeTries>=maxRetries, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            timer.reset();
        }, reverseToLeave);

        intake.addCondition(()->detector.getPixels() == 0 && timer.seconds() >= intakeTimeOut, ()->{
            robot.intake.setState(Intake.State.REVERSE);
            intakeTries++;
            timer.reset();
        }, reverseToRetry);

        reverseToRetry.addCondition(()->timer.seconds() >= reverseTime, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            timer.reset();
        }, intake);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == -1, ()->{
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(alignToCrossFieldForYellowPosition);
        }, alignToCross);

        reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle >= 0, ()->{
            robot.intake.setState(Intake.State.GOING_IN);
            drive.setTargetPose(scorePositions[cycle]);
        }, goToScoringPosition);



    }

}
