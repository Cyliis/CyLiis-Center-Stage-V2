package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class BluePositions {
    //Left

    public static Pose purplePosition1 = new Pose(35.5,4,1.93);
    public static int preloadIntakeExtendoPosition1 = 825;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(49.6,0.4,Math.PI/2);
    public static Pose crossFieldYellowPosition1 = new Pose(48.5,76.5,Math.PI/2);
    public static Pose scoreWhitePosition1 = new Pose(33,91.5,Math.PI/2);
    public static Pose scoreYellowPosition1 = new Pose(21.5,91.5,Math.PI/2);

    public static Pose alignToCrossBackPosition1 = new Pose(48.8,75.9,Math.PI/2);
    public static Pose[] intakePositions1 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance1 = {30,30,10,10};
    public static double[] extendoHeadingTolerance1 = {0.1,0.1,0.1,0.1};
    public static int[] extendoPositions1 = {0,0,0,0};
    public static Pose[] scorePositions1 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    public static Pose[] parkingPositions1 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    
    //Middle

    public static Pose purplePosition2 = new Pose(0,0,0);
    public static int preloadIntakeExtendoPosition2 = 0;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(0,0,0);
    public static Pose crossFieldYellowPosition2 = new Pose(0,0,0);
    public static Pose scoreYellowPosition2 = new Pose(0,0,0);
    public static Pose scoreWhitePosition2 = new Pose(0,0,0);

    public static Pose alignToCrossBackPosition2 = new Pose(0,0,0);
    public static Pose[] intakePositions2 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance2 = {30,30,10,10};
    public static double[] extendoHeadingTolerance2 = {0.1,0.1,0.1,0.1};
    public static int[] extendoPositions2 = {0,0,0,0};
    public static Pose[] scorePositions2 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    public static Pose[] parkingPositions2 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    
    //Right

    public static Pose purplePosition3 = new Pose(0,0,0);
    public static int preloadIntakeExtendoPosition3 = 0;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(0,0,0);
    public static Pose crossFieldYellowPosition3 = new Pose(0,0,0);
    public static Pose scoreYellowPosition3 = new Pose(0,0,0);
    public static Pose scoreWhitePosition3 = new Pose(0,0,0);

    public static Pose alignToCrossBackPosition3 = new Pose(0,0,0);
    public static Pose[] intakePositions3 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance3 = {30,30,10,10};
    public static double[] extendoHeadingTolerance3 = {0.1,0.1,0.1,0.1};
    public static int[] extendoPositions3 = {0,0,0,0};
    public static Pose[] scorePositions3 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
    public static Pose[] parkingPositions3 = {
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0,0)
    };
}
