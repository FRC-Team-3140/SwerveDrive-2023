package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED  extends SubsystemBase{
    static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(43);
    static AddressableLED led;
    static boolean yellowPurple;

    public LED(){
        led = new AddressableLED(1);
        led.setLength(ledBuffer.getLength());
        yellowPurple = false;
        led.start();
        
    }

    public void purple(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i, 200, 0,255);        
        }
        led.setData(ledBuffer);
        yellowPurple = false;
    }
    
    public void yellow(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i, 240, 179, 12);
        }//GOOD GOD
        led.setData(ledBuffer);
        yellowPurple = true;
    }

    public void toggleLED(){
        if(!yellowPurple){
            
            yellow();
        }else{
            
            purple();
        }
        
    }

    

    
}
