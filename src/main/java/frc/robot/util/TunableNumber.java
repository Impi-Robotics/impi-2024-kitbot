package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.Constants.Mode;

public class TunableNumber {
    private static final String tableKey = "TunableNumbers";

    private String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;

    public TunableNumber (String dashboardKey){
        this.key = tableKey + "/" + dashboardKey;
    }
    public TunableNumber(String dashboardKey, double defaultValue){
        this(dashboardKey);
        setDefault(defaultValue);
    }
    public double getDefault(){
        return defaultValue;
    }
    public void setDefault(double defaultValue){
        this.defaultValue = defaultValue;
        if(Mode.TUNING_MODE){
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        }
    }
    public double get(){
        return Mode.TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue)
            : defaultValue;
    }
    public boolean hasChanged(){
        double currentValue = get();
        if(currentValue != lastHasChangedValue){
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }
}
