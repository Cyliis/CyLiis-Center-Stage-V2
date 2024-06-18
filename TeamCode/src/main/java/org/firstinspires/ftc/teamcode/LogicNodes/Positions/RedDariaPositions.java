package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class RedDariaPositions {
    //Right

    public static Pose purplePosition3 = new Pose(29, -1.5, -1.787);
    public static int preloadIntakeExtendoPosition3 = 780;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(50, 0, -Math.PI / 2);
    public static Pose crossFieldYellowPosition3 = new Pose(50, -68, 4.365);
    public static Pose scoreYellowPosition3 = new Pose(30 , -88, 4.212);

    public static Pose alignToCrossBackPosition3 = new Pose(50, -75, -Math.PI / 2);
    public static Pose[] beforeIntakePositions3 = {
            new Pose(50.5, -75, -Math.PI / 2),
            new Pose(52.5, -75, -Math.PI / 2),
            new Pose(53, -75, -Math.PI / 2),
    };
    public static Pose[] intakePositions3 = {
            new Pose(50.5, -16.5, -Math.PI / 2),
            new Pose(52, -16.5, -Math.PI / 2),
            new Pose(53, -15.5, -1.3),
            new Pose(0, 0, 0)
    };
    public static double[] extendoDistanceTolerance3 = {60, 20, 15};
    public static double[] extendoHeadingTolerance3 = {0.025, 0.025, 0.04};
    public static int[] extendoPositions3 = {1320, 1320, 1320};
    public static Pose[] beforeScoringPositions3 = {
            new Pose(50, -75, -Math.PI / 2),
            new Pose(52, -75, -Math.PI / 2),
            new Pose(53, -75, -Math.PI / 2)
    };
    public static Pose[] scorePositions3 = {
            new Pose(44, -88, -Math.PI / 2 - Math.toRadians(30)),
            new Pose(44, -88.5, -Math.PI / 2 - Math.toRadians(30)),
            new Pose(44, -88.5, -Math.PI / 2 - Math.toRadians(30))
    };
    public static Pose[] parkingPositions3 = {
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2)
    };

    //Middle

    public static Pose purplePosition2 = new Pose(36, 11, 4.718);
    public static int preloadIntakeExtendoPosition2 = 230;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(50, 10, -Math.PI / 2);
    public static Pose crossFieldYellowPosition2 = new Pose(50, -68, 4.365);
    public static Pose scoreYellowPosition2 = new Pose(33, -88, 4.212);

    public static Pose alignToCrossBackPosition2 = new Pose(50, -75, -Math.PI / 2);
    public static Pose[] beforeIntakePositions2 = {
            new Pose(49.5, -75, -Math.PI / 2),
            new Pose(51.5, -75, -Math.PI / 2),
            new Pose(52, -75, -Math.PI / 2),
    };
    public static Pose[] intakePositions2 = {
            new Pose(49.5, -16.5, -Math.PI / 2),
            new Pose(51.5, -16.5, -Math.PI / 2),
            new Pose(52, -15, -1.3),
            new Pose(0, 0, 0)
    };
    public static double[] extendoDistanceTolerance2 = {60, 20, 15};
    public static double[] extendoHeadingTolerance2 = {0.025, 0.025, 0.04};
    public static int[] extendoPositions2 = {1320, 1320, 1320};
    public static Pose[] beforeScoringPositions2 = {
            new Pose(50, -75, -Math.PI / 2),
            new Pose(52, -75, -Math.PI / 2),
            new Pose(53, -75, -Math.PI / 2)
    };
    public static Pose[] scorePositions2 = {
            new Pose(44, -88.5, -Math.PI / 2 - Math.toRadians(30)),
            new Pose(44, -88.5, -Math.PI / 2 - Math.toRadians(30)),
            new Pose(44, -88.5, -Math.PI / 2 - Math.toRadians(30))
    };
    public static Pose[] parkingPositions2 = {
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2)
    };

    //Left

    public static Pose purplePosition1 = new Pose(31, 19, 4.1);
    public static int preloadIntakeExtendoPosition1 = 120;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(50, 6, -Math.PI / 2);
    public static Pose crossFieldYellowPosition1 = new Pose(50, -68, 4.365);
    public static Pose scoreYellowPosition1 = new Pose(37.5, -88, 4.212);

    public static Pose alignToCrossBackPosition1 = new Pose(50, -75, -Math.PI / 2);
    public static Pose[] beforeIntakePositions1 = {
            new Pose(48.5, -75, -Math.PI / 2),
            new Pose(50.5, -75, -Math.PI / 2),
            new Pose(51, -75, -Math.PI / 2),
    };
    public static Pose[] intakePositions1 = {
            new Pose(48.5, -16.5, -Math.PI / 2),
            new Pose(50.5, -16.5, -Math.PI / 2),
            new Pose(51, -15, -1.3),
            new Pose(0, 0, 0)
    };
    public static double[] extendoDistanceTolerance1 = {60, 20, 15};
    public static double[] extendoHeadingTolerance1 = {0.025, 0.025, 0.04};
    public static int[] extendoPositions1 = {1320, 1320, 1320};
    public static Pose[] beforeScoringPositions1 = {
            new Pose(50, -75, -Math.PI / 2),
            new Pose(52, -75, -Math.PI / 2),
            new Pose(53, -75, -Math.PI / 2)
    };
    public static Pose[] scorePositions1 = {
            new Pose(42, -88.5, -Math.PI / 2 - Math.toRadians(30)),
            new Pose(42, -88.5, -Math.PI / 2 - Math.toRadians(30)),
            new Pose(42, -88.5, -Math.PI / 2 - Math.toRadians(30))
    };
    public static Pose[] parkingPositions1 = {
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2),
            new Pose(38, -83, -Math.PI / 2)
    };
}
