package org.firstinspires.ftc.teamcode.LogicNodes.Positions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class BlueDariaPositions {
    //Left

    public static Pose purplePosition1 = new Pose(31.5,2.5,1.787);
    public static int preloadIntakeExtendoPosition1 = 820;

    public static Pose alignToCrossFieldForYellowPosition1 = new Pose(52,0,Math.PI/2);
    public static Pose crossFieldYellowPosition1 = new Pose(51,68,-4.365);
    public static Pose scoreYellowPosition1 = new Pose(34,89,-4.212);

    public static Pose alignToCrossBackPosition1 = new Pose(51,75,Math.PI/2);
    public static Pose[] beforeIntakePositions1 = {
            new Pose(52.5, 75, Math.PI/2),
            new Pose(52.5, 75, Math.PI/2),
            new Pose(55, 75, Math.PI/2),
    };
    public static Pose[] intakePositions1 = {
            new Pose(51.5,17,Math.PI/2),
            new Pose(53.5,17,Math.PI/2),
            new Pose(55.5,16,1.25),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance1 = {60,20,10};
    public static double[] extendoHeadingTolerance1 = {0.05,0.05,0.05};
    public static int[] extendoPositions1 = {1320,1320,1320};
    public static Pose[] beforeScoringPositions1 = {
            new Pose(51.5,75,Math.PI/2),
            new Pose(53,75,Math.PI/2),
            new Pose(55,75,Math.PI/2)
    };
    public static Pose[] scorePositions1 = {
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30)),
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30)),
            new Pose(46,89,Math.PI/2 + Math.toRadians(30))
    };
    public static Pose[] parkingPositions1 = {
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2)
    };

    //Middle

    public static Pose purplePosition2 = new Pose(38.5,-13,Math.PI/2);
    public static int preloadIntakeExtendoPosition2 = 280;

    public static Pose alignToCrossFieldForYellowPosition2 = new Pose(52,-10,Math.PI/2);
    public static Pose crossFieldYellowPosition2 = new Pose(52,68,-4.365);
    public static Pose scoreYellowPosition2 = new Pose(39,89.5,-4.212);

    public static Pose alignToCrossBackPosition2 = new Pose(51,75,Math.PI/2);
    public static Pose[] beforeIntakePositions2 = {
            new Pose(52, 75, Math.PI/2),
            new Pose(52.5, 75, Math.PI/2),
            new Pose(55, 75, Math.PI/2),
    };
    public static Pose[] intakePositions2 = {
            new Pose(51.5,17,Math.PI/2),
            new Pose(53.5,17,Math.PI/2),
            new Pose(55.5,16,1.25),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance2 = {60,20,10};
    public static double[] extendoHeadingTolerance2 = {0.05,0.05,0.05};
    public static int[] extendoPositions2 = {1320,1320,1320};
    public static Pose[] beforeScoringPositions2 = {
            new Pose(51.5,75,Math.PI/2),
            new Pose(53,75,Math.PI/2),
            new Pose(55,75,Math.PI/2)
    };
    public static Pose[] scorePositions2 = {
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30)),
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30)),
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30))
    };
    public static Pose[] parkingPositions2 = {
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2)
    };


    //Right

    public static Pose purplePosition3 = new Pose(32.5,-17, -4.2);
    public static int preloadIntakeExtendoPosition3 = 160;

    public static Pose alignToCrossFieldForYellowPosition3 = new Pose(52,-6,Math.PI/2);
    public static Pose crossFieldYellowPosition3 = new Pose(52,68,-4.365);
    public static Pose scoreYellowPosition3 = new Pose(41.5,89.5,-4.212);

    public static Pose alignToCrossBackPosition3 = new Pose(51,75,Math.PI/2);
    public static Pose[] beforeIntakePositions3 = {
            new Pose(51, 75, Math.PI/2),
            new Pose(52.5, 75, Math.PI/2),
            new Pose(55, 75, Math.PI/2),
    };
    public static Pose[] intakePositions3 = {
            new Pose(51,17,Math.PI/2),
            new Pose(52.5,17,Math.PI/2),
            new Pose(55,16,1.25),
            new Pose(0,0,0)
    };
    public static double[] extendoDistanceTolerance3 = {60,20,10};
    public static double[] extendoHeadingTolerance3 = {0.05,0.05,0.05};
    public static int[] extendoPositions3 = {1320,1320,1320};
    public static Pose[] beforeScoringPositions3 = {
            new Pose(51.5,75,Math.PI/2),
            new Pose(53,75,Math.PI/2),
            new Pose(55,75,Math.PI/2)
    };
    public static Pose[] scorePositions3 = {
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30)),
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30)),
            new Pose(46,89.5,Math.PI/2 + Math.toRadians(30))
    };
    public static Pose[] parkingPositions3 = {
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2),
            new Pose(38,83,Math.PI/2)
    };
}
