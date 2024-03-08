package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class RedPositions {
    //Right

    public static Pose purplePosition3 = new Pose(30,-2.5,-1.787);
    public static int preloadIntakeExtendoPosition3 = 760;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(51,6,-Math.PI/2);
    public static Pose crossFieldYellowPosition3 = new Pose(51,-75,-Math.PI/2);
    public static Pose scoreWhitePosition3 = new Pose(30,-89.5,-Math.PI/2);
    public static Pose scoreYellowPosition3 = new Pose(18,-89.5,-Math.PI/2);

    public static Pose alignToCrossBackPosition3 = new Pose(51,-75,-Math.PI/2);
    public static Pose[] intakePositions3 = {
            new Pose(49.5,-12,-Math.PI/2),
            new Pose(52,-11.5,-Math.PI/2),
            new Pose(54,-12,-1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance3 = {45,45,10};
    public static double[] extendoHeadingTolerance3 = {0.03,0.03,0.05};
    public static int[] extendoPositions3 = {1100,1100,1130};
    public static Pose[] scorePositions3 = {
            new Pose(47,-89,-2.34),
            new Pose(47,-89,-2.34),
            new Pose(47,-92,-2.34)
    };
    public static Pose[] parkingPositions3 = {
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2),
            new Pose(38,-81,-Math.PI/2)
    };

    //Middle

    public static Pose purplePosition2 = new Pose(37.5,12,-1.593);
    public static int preloadIntakeExtendoPosition2 = 260;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(52,4,-Math.PI/2);
    public static Pose crossFieldYellowPosition2 = new Pose(52,-75,-Math.PI/2);
    public static Pose scoreWhitePosition2 = new Pose(18,-90.5,-Math.PI/2);
    public static Pose scoreYellowPosition2 = new Pose(25,-90.5,-Math.PI/2);

    public static Pose alignToCrossBackPosition2 = new Pose(52,-75,-Math.PI/2);
    public static Pose[] intakePositions2 = {
            new Pose(48.5,-11.5,-Math.PI/2),
            new Pose(51,-12,-Math.PI/2),
            new Pose(53,-12,-1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance2 = {45,45,10};
    public static double[] extendoHeadingTolerance2 = {0.05,0.05,0.05};
    public static int[] extendoPositions2 = {1100,1100,1130};
    public static Pose[] scorePositions2 = {
            new Pose(46,-89.5,-2.34),
            new Pose(47,-89,-2.34),
            new Pose(53,-91.5,-2.34)
    };
    public static Pose[] parkingPositions2 = {
            new Pose(40,-81,-Math.PI/2),
            new Pose(40,-81,-Math.PI/2),
            new Pose(40,-81,-Math.PI/2),
            new Pose(40,-81,-Math.PI/2)
    };


    //Left

    public static Pose purplePosition1 = new Pose(32,18,-1.97);
    public static int preloadIntakeExtendoPosition1 = 100;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(52,4,-Math.PI/2);
    public static Pose crossFieldYellowPosition1 = new Pose(52,-75,-Math.PI/2);
    public static Pose scoreWhitePosition1 = new Pose(23,-91,-Math.PI/2);
    public static Pose scoreYellowPosition1 = new Pose(29,-90.5,-Math.PI/2);

    public static Pose alignToCrossBackPosition1 = new Pose(52,-75,-Math.PI/2);
    public static Pose[] intakePositions1 = {
            new Pose(47.5,-11.5,-Math.PI/2),
            new Pose(51,-12,-Math.PI/2),
            new Pose(53,-12,-1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance1 = {45,45,10};
    public static double[] extendoHeadingTolerance1 = {0.05,0.05,0.05};
    public static int[] extendoPositions1 = {1100,1100,1130};
    public static Pose[] scorePositions1 = {
            new Pose(44,-89.5,-2.34),
            new Pose(46,-89,-2.34),
            new Pose(53,-91.5,-2.34)
    };
    public static Pose[] parkingPositions1 = {
            new Pose(40,-81,-Math.PI/2),
            new Pose(40,-81,-Math.PI/2),
            new Pose(40,-81,-Math.PI/2),
            new Pose(40,-81,-Math.PI/2)
    };
}
