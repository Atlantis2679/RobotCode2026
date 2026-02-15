package frc.robot.utils;

public class MathUtils {
    public static double avg(double[] values) {
        if (values == null || values.length == 0) {
            return 0.0;
        }
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.length;
    }
    public static int avg(int[] values) {
        if (values == null || values.length == 0) {
            return 0;
        }
        long sum = 0;
        for (int v : values) {
            sum += v;
        }
        return Math.round(sum / values.length);
    }
}
