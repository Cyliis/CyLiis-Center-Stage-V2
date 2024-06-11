package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class BlueDariaPositions {
    //Left

    public static Pose purplePosition1 = new Pose(32,2,1.787);
    public static int preloadIntakeExtendoPosition1 = 790;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(52,0,Math.PI/2);
    public static Pose crossFieldYellowPosition1 = new Pose(52,70,Math.PI/2);
    public static Pose scoreWhitePosition1 = new Pose(36,86.5,Math.PI/2);
    public static Pose scoreYellowPosition1 = new Pose(25,87,Math.PI/2);

    public static Pose alignToCrossBackPosition1 = new Pose(52,70,Math.PI/2);
    public static Pose[] intakePositions1 = {
            new Pose(52.5,10,Math.PI/2),
            new Pose(54,10,Math.PI/2),
            new Pose(56.5,8.5,1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance1 = {50,50,20};
    public static double[] extendoHeadingTolerance1 = {0.03,0.03,0.05};
    public static int[] extendoPositions1 = {1100,1100,1115};
    public static Pose[] scorePositions1 = {
            new Pose(52,91,2.34),
            new Pose(52.5,91,2.34),
            new Pose(54,91,2.34)
    };
    public static Pose[] parkingPositions1 = {
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2)
    };

    //Middle

    public static Pose purplePosition2 = new Pose(38.5,-13,1.593);
    public static int preloadIntakeExtendoPosition2 = 250;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(52,-4,Math.PI/2);
    public static Pose crossFieldYellowPosition2 = new Pose(52,73,Math.PI/2);
    public static Pose scoreWhitePosition2 = new Pose(24,87.5,Math.PI/2);
    public static Pose scoreYellowPosition2 = new Pose(32.5,87.5,Math.PI/2);

    public static Pose alignToCrossBackPosition2 = new Pose(52,73,Math.PI/2);
    public static Pose[] intakePositions2 = {
            new Pose(53.5,9.5,Math.PI/2),
            new Pose(56,9.5,Math.PI/2),
            new Pose(57.5,10,1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance2 = {50,50,20};
    public static double[] extendoHeadingTolerance2 = {0.03,0.03,0.05};
    public static int[] extendoPositions2 = {1100,1100,1130};
    public static Pose[] scorePositions2 = {
            new Pose(53,90,2.34),
            new Pose(54.5,90,2.34),
            new Pose(55,90,2.34)
    };
    public static Pose[] parkingPositions2 = {
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2)
    };


    //Right

    public static Pose purplePosition3 = new Pose(32,-18, -4.258);
    public static int preloadIntakeExtendoPosition3 = 100;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(52,-6,Math.PI/2);
    public static Pose crossFieldYellowPosition3 = new Pose(52,73,Math.PI/2);
    public static Pose scoreWhitePosition3 = new Pose(31,87.5,Math.PI/2);
    public static Pose scoreYellowPosition3 = new Pose(39,87.5,Math.PI/2);

    public static Pose alignToCrossBackPosition3 = new Pose(52,73,Math.PI/2);
    public static Pose[] intakePositions3 = {
            new Pose(52,10,Math.PI/2),
            new Pose(54,11.5,Math.PI/2),
            new Pose(55.5,9,1.3),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance3 = {50,50,20};
    public static double[] extendoHeadingTolerance3 = {0.03,0.03,0.05};
    public static int[] extendoPositions3 = {1100,1100,1130};
    public static Pose[] scorePositions3 = {
            new Pose(53,90.5,2.34),
            new Pose(53,90.5,2.34),
            new Pose(53,90.5,2.34)
    };
    public static Pose[] parkingPositions3 = {
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2),
            new Pose(40,81,Math.PI/2)
    };
}
