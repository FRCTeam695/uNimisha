package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

    // Creates LED and AdressableLED object

    public final AddressableLED myLed = new AddressableLED(1);
    public final AddressableLEDBuffer myLedBuffer;

    public int rv;
    public int gv;
    public int bv;
    
    public LEDSubsystem() {
        
        // Initialize LED and buffer
        
        myLedBuffer = new AddressableLEDBuffer(5);
        myLed.setLength(myLedBuffer.getLength());
        myLed.setData(myLedBuffer);
        myLed.start();

    }

    public enum Color {
        BLUE,
        CYAN,
        GREEN,
        YELLOW,
        ORANGE,
        RED,
        PINK,
        PURPLE,
        COLORLESS
    }

    public class ChooseColor {
        
        Color myColor;

        public ChooseColor (Color color) {
            myColor = color;
        }

        public void setColor() {
            switch(myColor) {
                case BLUE: 
                rv = 0;
                gv = 0;
                bv = 255;
                break;
                
                case CYAN:
                rv = 0;
                gv = 255;
                bv = 255;
                break;

                case GREEN:
                rv = 0;
                gv = 255;
                bv = 0;
                break;
                
                case YELLOW:
                rv = 255;
                gv = 255;
                bv = 0;
                break;
                
                case ORANGE:
                rv = 255;
                gv = 128;
                bv = 13;
                break;
                
                case RED:
                rv = 255;
                gv = 0;
                bv = 0;
                break;
                
                case PINK:
                rv = 255;
                gv = 192;
                bv = 203;
                break;

                case PURPLE:
                rv = 156;
                gv = 81;
                bv = 182;
                break;

                case COLORLESS:
                rv = 0;
                gv = 0;
                bv = 0;
                break;
            }
        }
    }

    public Command changeColorCommand (Color choice, double brightness) {

        return new FunctionalCommand(

            // INIT
            () -> {},

            // EXECUTE
            () -> {
                ChooseColor color = new ChooseColor(choice);
                color.setColor();
                for (int i = 0; i < myLedBuffer.getLength(); i++) {
                    myLedBuffer.setRGB(i, (int)(rv*brightness), (int)(gv*brightness), (int)(bv*brightness));
                }
                myLed.setData(myLedBuffer);
            },

            interrupted -> {},

            () -> false,

        this);
    }

    public Command changeColorDirection (Color pChoice, Color nChoice, double brightness, double speed) {

        return new FunctionalCommand(

            // INIT
            () -> {},

            // EXECUTE
            () -> {
                double s = speed;
                double b = brightness;
                if (s > 0) {
                    ChooseColor posColor = new ChooseColor(pChoice);
                    posColor.setColor();
                }
                if (s < 0) {
                    ChooseColor negColor = new ChooseColor(nChoice);
                    negColor.setColor();
                }
                if (s == 0) {
                    new ChooseColor(Color.COLORLESS).setColor();
                }
                
                for (int i = 0; i < myLedBuffer.getLength(); i++) {
                    myLedBuffer.setRGB(i, (int)(Math.abs(rv*b)), (int)(Math.abs(gv*b)), (int)(Math.abs(bv*b)));
                }
                myLed.setData(myLedBuffer);
            },

            interrupted -> {},

            () -> false,

        this);
    }

  public void changeColor(Color choice, double brightness) {
    ChooseColor color = new ChooseColor(choice);
    color.setColor();
    for (int i = 0; i < myLedBuffer.getLength(); i++) {
      myLedBuffer.setRGB(i, (int)(rv*brightness), (int)(gv*brightness), (int)(bv*brightness));
    }
    myLed.setData(myLedBuffer);
  }
}
