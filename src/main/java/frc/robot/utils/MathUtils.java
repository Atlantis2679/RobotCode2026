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

    public static double avg(double... values) {
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.length;
    }

    public static double median(double... arr) {
        int n = arr.length;
        Arrays.sort(arr);
        if (n % 2 != 0) {
          return arr[n / 2];
        }
        return avg(arr[n / 2], arr[n / 2 + 1]);
    }

    public static boolean inRange(double value, double low, double high) {
        return value >= low && value <= high;
    }
}
