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

    public static class DynamicAvarage {
        private Double[] values;
        private int i = 0;
        private boolean isEmpty = true;
        
        public DynamicAvarage(int len) {
            values = new Double[len];
        }

        public void update(Double val) {
            values[i] = val;
            ++i;
            if (i>values.length) {
                i=0;
            }
            isEmpty = false;
        }

        public Double get() {
            Double sum = 0.0;
            int len = values.length;
            for (Double num : values) {
                if (num == null) {
                    --len;
                } else {
                    sum += num;
                }
            }
            if (len==0) {
                return 0.0;
            }
            return sum/len;
        }

        public void reset() {
            if (!isEmpty) {
                this.values = new Double[values.length];
                i = 0;
                isEmpty = true;
            }
        }
    }

    public static double[] getHighestX(int x, double[] vals) {
        java.util.Arrays.sort(vals);
        double[] res = new double[x];
        for (int i = 0; i < x; i++) {
            res[i] = vals[vals.length-1-i];
        }
        return res;
    }
}
