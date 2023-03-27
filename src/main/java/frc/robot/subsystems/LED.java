package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED  extends SubsystemBase{
    static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(44);
    static AddressableLED led;
    static boolean yellowPurple;
    private int m_rainbowFirstPixelHue = 0;
    public boolean runRainbow = true;
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
        runRainbow = false;
        rainbow();
        if(!yellowPurple){
            
            yellow();
        }else{
            
            purple();
        }
        
    }


    public void flagship(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            if(i % 2 == 0){
                ledBuffer.setRGB(i, 195, 193, 193);
                
            }else {
                ledBuffer.setRGB(i, 1, 12, 22); 
            }
            
            
        }
        led.setData(ledBuffer);
    }

    public void rainbow() {
        if(runRainbow) {
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                int hue;
                if (i<55){
                    hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180; 
                    ledBuffer.setHSV(i, hue, 255, 128);
                }
                else
                {
                    hue=0;
                    if ((m_rainbowFirstPixelHue%20)<10)
                    {
                        ledBuffer.setHSV(i, hue, 255, 0);
                    }
                    else
                    {
                        ledBuffer.setHSV(i, hue, 255, 128);
                    }
            
                }
            }
            m_rainbowFirstPixelHue += 3;
            m_rainbowFirstPixelHue %= 180;
            led.setData(ledBuffer);
        }
    }

    public void leftRightCenter(double distanceFromCenter){
        if(distanceFromCenter > 0){
           // int AmountOfLEDSidk = 9;
            

            ledBuffer.setRGB(3, 240, 179, 12);

        }
        
    }

    

    
}
