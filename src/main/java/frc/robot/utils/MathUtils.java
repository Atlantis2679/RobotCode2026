package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;

public class MathUtils {
    public static double cosineWave(double max, double min, double time) {
        double average = (max + min) / 2;
        double delta = (max - min) / 2;
        return average + delta * Math.cos(time);
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

    public static class RangeWrapper {
        private final double lowerBound;
        private final double rangeLength;

        public RangeWrapper(double lowerBound, double upperBound) {
            this.lowerBound = lowerBound;
            this.rangeLength = upperBound - lowerBound;
        }

        public double wrap(double value) {
            return ((value - lowerBound) % rangeLength + rangeLength) % rangeLength + lowerBound;
        }
    }

    public static class NumberGoalTester {
        public final double target;
        public final boolean positiveDir;
        private final Optional<RangeWrapper> rangeWrapper;
        private final Debouncer goalReachedDebouncer;

        public NumberGoalTester(double target, boolean positiveDir, Optional<RangeWrapper> rangeWrapper, double debounceSec) {
            if (rangeWrapper.isPresent()) {
                target = rangeWrapper.get().wrap(target);    
            }
            this.target = target;
            this.positiveDir = positiveDir;
            this.rangeWrapper = rangeWrapper;
            this.goalReachedDebouncer = new Debouncer(debounceSec);
        }

        public boolean goalReached(double current) {
            if (rangeWrapper.isPresent()) {
                current = rangeWrapper.get().wrap(current);    
            }
            return goalReachedDebouncer.calculate(positiveDir ? current >= target : current <= target);
        }
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
            if (i>=values.length) {
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
