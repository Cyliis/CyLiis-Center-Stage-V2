package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

public class DashboardUtil {
    private static final double ROBOT_X_FRONT = 403.0/25.4/2.0;
    private static final double ROBOT_X_REAR = 403.0/25.4/2.0;
    private static final double ROBOT_Y = 340.0/25.4;

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        Vector2d[] corners = new Vector2d[] {
                new Vector2d(ROBOT_X_FRONT, ROBOT_Y / 2).rotated(pose.getHeading()).plus(pose.vec()),
                new Vector2d(ROBOT_X_FRONT, -ROBOT_Y / 2).rotated(pose.getHeading()).plus(pose.vec()),
                new Vector2d(-ROBOT_X_REAR, -ROBOT_Y / 2).rotated(pose.getHeading()).plus(pose.vec()),
                new Vector2d(-ROBOT_X_REAR, ROBOT_Y / 2).rotated(pose.getHeading()).plus(pose.vec()),
        };
        double[] xPoints = new double[corners.length];
        double[] yPoints = new double[corners.length];
        for(int i = 0; i < corners.length; i++) {
            xPoints[i] = corners[i].getX();
            yPoints[i] = corners[i].getY();
        }
        canvas.strokePolygon(xPoints, yPoints);
        Vector2d v = pose.headingVec().times(ROBOT_X_FRONT);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}