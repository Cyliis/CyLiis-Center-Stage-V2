package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class RedPositions {
    //Right

    public static Pose purplePosition3 = new Pose(29,-2.5,-1.787);
    public static int preloadIntakeExtendoPosition3 = 740;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(51,0,-Math.PI/2);
    public static Pose crossFieldYellowPosition3 = new Pose(51,-72,-Math.PI/2);
    public static Pose scoreWhitePosition3 = new Pose(30,-90,-Math.PI/2);
    public static Pose scoreYellowPosition3 = new Pose(17.5,-90,-Math.PI/2);

    public static Pose alignToCrossBackPosition3 = new Pose(51,-75,-Math.PI/2);
    public static Pose[] intakePositions3 = {
            new Pose(48.5,-13,-Math.PI/2),
            new Pose(50.5,-13,-Math.PI/2),
            new Pose(53.5,-12,-1.2),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance3 = {50,50,20};
    public static double[] extendoHeadingTolerance3 = {0.03,0.03,0.05};
    public static int[] extendoPositions3 = {1100,1100,1180};
    public static Pose[] scorePositions3 = {
            new Pose(47,-91.5,-2.34),
            new Pose(47,-92,-2.34),
            new Pose(47,-90.5,-2.34)
    };
    public static Pose[] parkingPositions3 = {
            new Pose(38,-85,-Math.PI/2),
            new Pose(38,-85,-Math.PI/2),
            new Pose(38,-85,-Math.PI/2),
            new Pose(38,-85,-Math.PI/2)
    };

    //Middle

    public static Pose purplePosition2 = new Pose(36,14,4.718);
    public static int preloadIntakeExtendoPosition2 = 240;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(52,10,-Math.PI/2);
    public static Pose crossFieldYellowPosition2 = new Pose(52,-68,4.365);
    public static Pose scoreYellowPosition2 = new Pose(36.5,-88,4.212);

    public static Pose alignToCrossBackPosition2 = new Pose(51,-75,-Math.PI/2);
    public static Pose[] intakePositions2 = {
            new Pose(50,-16,-Math.PI/2),
            new Pose(52,-15,-Math.PI/2),
            new Pose(53,-16,-1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance2 = {60,60,15};
    public static double[] extendoHeadingTolerance2 = {0.025,0.025,0.04};
    public static int[] extendoPositions2 = {1320,1320,1320};
    public static Pose[] scorePositions2 = {
            new Pose(51,-91,-2.34),
            new Pose(51.5,-91,-2.34),
            new Pose(52,-91.5,-2.34)
    };
    public static Pose[] parkingPositions2 = {
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2)
    };


    //Left

    public static Pose purplePosition1 = new Pose(31,19, 4.258);
    public static int preloadIntakeExtendoPosition1 = 100;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(52,6,-Math.PI/2);
    public static Pose crossFieldYellowPosition1 = new Pose(52,-68,4.365);
    public static Pose scoreYellowPosition1 = new Pose(39.5,-88,4.212);

    public static Pose alignToCrossBackPosition1 = new Pose(51,-75,-Math.PI/2);
    public static Pose[] intakePositions1 = {
            new Pose(50,-16,-Math.PI/2),
            new Pose(52.5,-15.5,-Math.PI/2),
            new Pose(53.5,-16,-1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance1 = {60,60,15};
    public static double[] extendoHeadingTolerance1 = {0.025,0.025,0.04};
    public static int[] extendoPositions1 = {1320,1320,1320};
    public static Pose[] scorePositions1 = {
            new Pose(50.5,-91,-2.34),
            new Pose(51,-91,-2.34),
            new Pose(51.5,-91.5,-2.34)
    };
    public static Pose[] parkingPositions1 = {
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2)
    };
}
