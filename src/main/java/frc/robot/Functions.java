package frc.robot;

public class Functions {
    public static double angleDiff(double angle1, double angle2) {
        while (angle1 < 0) {
            angle1 += 360;
        }
        while (angle1 >= 360) {
            angle1 -= 360;
        }
        while (angle2 < 0) {
            angle2 += 360;
        }
        while (angle2 >= 360) {
            angle2 -= 360;
        }

        double diff = angle1 - angle2;
        if (diff > 180) {
            diff -= 360;
        }
        if (diff <= -180) {
            diff += 360;
        }

        return diff;
    }
}
