package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class BluePositions {
    //Left

    public static Pose purplePosition1 = new Pose(32,2,1.787);
    public static int preloadIntakeExtendoPosition1 = 770;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(52,-4,Math.PI/2);
    public static Pose crossFieldYellowPosition1 = new Pose(52,75,Math.PI/2);
    public static Pose scoreWhitePosition1 = new Pose(33,90,Math.PI/2);
    public static Pose scoreYellowPosition1 = new Pose(21.5,90,Math.PI/2);

    public static Pose alignToCrossBackPosition1 = new Pose(52,75,Math.PI/2);
    public static Pose[] intakePositions1 = {
            new Pose(51,10,Math.PI/2),
            new Pose(53,10,Math.PI/2),
            new Pose(55,12,1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance1 = {45,45,10};
    public static double[] extendoHeadingTolerance1 = {0.05,0.05,0.05};
    public static int[] extendoPositions1 = {1100,1100,1115};
    public static Pose[] scorePositions1 = {
            new Pose(50.5,91,2.34),
            new Pose(50.5,91,2.34),
            new Pose(50.5,92,2.34)
    };
    public static Pose[] parkingPositions1 = {
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2)
    };

    //Middle

    public static Pose purplePosition2 = new Pose(37.5,-12,1.593);
    public static int preloadIntakeExtendoPosition2 = 280;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(52,-4,Math.PI/2);
    public static Pose crossFieldYellowPosition2 = new Pose(52,75,Math.PI/2);
    public static Pose scoreWhitePosition2 = new Pose(24,90,Math.PI/2);
    public static Pose scoreYellowPosition2 = new Pose(29,90,Math.PI/2);

    public static Pose alignToCrossBackPosition2 = new Pose(52,75,Math.PI/2);
    public static Pose[] intakePositions2 = {
            new Pose(51.5,10,Math.PI/2),
            new Pose(53,10,Math.PI/2),
            new Pose(53,12,1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance2 = {45,45,10};
    public static double[] extendoHeadingTolerance2 = {0.05,0.05,0.05};
    public static int[] extendoPositions2 = {1100,1100,1130};
    public static Pose[] scorePositions2 = {
            new Pose(50,91,2.34),
            new Pose(50,91,2.34),
            new Pose(53,91.5,2.34)
    };
    public static Pose[] parkingPositions2 = {
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2)
    };


    //Right

    public static Pose purplePosition3 = new Pose(32,-18,1.97);
    public static int preloadIntakeExtendoPosition3 = 100;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(52,-4,Math.PI/2);
    public static Pose crossFieldYellowPosition3 = new Pose(52,75,Math.PI/2);
    public static Pose scoreWhitePosition3 = new Pose(28,90.5,Math.PI/2);
    public static Pose scoreYellowPosition3 = new Pose(35,90.5,Math.PI/2);

    public static Pose alignToCrossBackPosition3 = new Pose(52,75,Math.PI/2);
    public static Pose[] intakePositions3 = {
            new Pose(52,11,Math.PI/2),
            new Pose(52.5,11,Math.PI/2),
            new Pose(54,12,1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance3 = {45,45,10};
    public static double[] extendoHeadingTolerance3 = {0.05,0.05,0.05};
    public static int[] extendoPositions3 = {1100,1100,1130};
    public static Pose[] scorePositions3 = {
            new Pose(49,91,2.34),
            new Pose(49,91,2.34),
            new Pose(49,91.5,2.34)
    };
    public static Pose[] parkingPositions3 = {
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2)
    };
}
