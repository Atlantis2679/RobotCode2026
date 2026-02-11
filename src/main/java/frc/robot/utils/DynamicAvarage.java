package frc.robot.utils;

public class DynamicAvarage {
    Double[] values;
    int i = 0;
    
    public DynamicAvarage(int len) {
        values = new Double[len];
    }

    public void update(Double val) {
        values[i] = val;
        ++i;
        if (i>values.length) {
            i=0;
        }
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
}
