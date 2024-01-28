package frc.robot.libs;

import com.ctre.phoenix6.hardware.CANcoder;

public class AbsoluteEncoder extends CANcoder {
    public int analogID;
    private double offset = 0;
    public AbsoluteEncoder(int analogID){
        super(analogID);

        this.analogID = analogID;

    }

     public void setPositionOffset(double offset){
        this.offset = offset;
    }
    public double getPositionOffset(int analogID){
        return this.offset;
    }

    public double getPos() {
        double pos = super.getAbsolutePosition().getValueAsDouble();
        return (pos)*360 - offset;
    }
    
    public double getPosRadians(){
        return super.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - offset * Math.PI/180;
    }
   
    
}
