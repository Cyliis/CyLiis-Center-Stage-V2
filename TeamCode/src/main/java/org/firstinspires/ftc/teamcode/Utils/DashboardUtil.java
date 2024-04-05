package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;

import java.util.ArrayList;
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

    public static double dt = 0.001;

    public static void drawPath(Canvas canvas, Trajectory trajectory){
        ArrayList<Double> pX = new ArrayList<>(), pY = new ArrayList<>();
        for(double i = 0; i <= 1;i+= dt){
            Pose p = trajectory.getPose(i);
            pX.add(p.getX());
            pY.add(p.getY());
        }

        double[] pointsX = new double[pX.size()];
        double[] pointsY = new double[pY.size()];

        for(int i = 0;i<pX.size();i++){
            pointsX[i] = pX.get(i);
            pointsY[i] = pY.get(i);
        }
        canvas.strokePolyline(pointsX, pointsY);
    }

    private final ArrayList<Pose> poseHistory = new ArrayList<>();

    public void clearPoseHistory(){
        poseHistory.clear();
    }

    public void drawPoseHistory(Canvas canvas, Pose pose){
        if(poseHistory.get(poseHistory.size() - 1) != pose) poseHistory.add(pose);

        double[] pointsX = new double[poseHistory.size()];
        double[] pointsY = new double[poseHistory.size()];

        for(int i = 0;i<poseHistory.size();i++){
            pointsX[i] = poseHistory.get(i).getX();
            pointsY[i] = poseHistory.get(i).getY();
        }
        canvas.strokePolyline(pointsX, pointsY);

    }
}