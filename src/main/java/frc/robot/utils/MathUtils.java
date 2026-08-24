package frc.robot.utils;

import java.util.Arrays;

import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

public class MathUtils {
    public static class CosineWaveFollower implements Tunable {
        public double min, max, speed, timestamp;

        public CosineWaveFollower(double min, double max, double speed) {
            this.min = min;
            this.max = max;
            this.speed = speed;
            this.timestamp = 0;
        }

        public CosineWaveFollower(double min, double max) {
            this(min, max, Math.PI / 180);
        }

        public double getNext() {
            timestamp += speed;
            return cosineWave(timestamp, min, max);
        }

        public static double cosineWave(double timestamp, double min, double max) {
            double average = (max + min) / 2;
            double delta = (max - min) / 2;
            return average + delta * Math.cos(timestamp);
        }

        @Override
        public void initTunable(TunableBuilder builder) {
            builder.addDoubleProperty("min",  () -> this.min, (min) -> this.min = min);
            builder.addDoubleProperty("max",  () -> this.max, (max) -> this.max = max);
            builder.addDoubleProperty("speed", () -> this.speed, (speed) -> this.speed = speed);
        }
    }

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

    public static class DynamicAverage {
        private double[] values;
        private int i = 0;
        private boolean isEmpty = true;
        
        public DynamicAverage(int len) {
            values = new double[len];
            Arrays.fill(values, Double.NaN);
        }

        public void update(double val) {
            values[i] = val;
            ++i;
            if (i>=values.length) {
                i=0;
            }
            isEmpty = false;
        }

        public double get() {
            double sum = 0.0;
            int len = values.length;
            for (double num : values) {
                if (Double.isNaN(num)) {
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
                this.values = new double[values.length];
                Arrays.fill(values, Double.NaN);
                i = 0;
                isEmpty = true;
            }
        }
    }

    public static double[] getHighestX(int x, double[] vals) {
        double[] sorted = vals.clone();
        Arrays.sort(sorted);
        double[] res = new double[x];
        for (int i = 0; i < x; i++) res[i] = sorted[sorted.length - 1 - i];
        return res;
    }
}
