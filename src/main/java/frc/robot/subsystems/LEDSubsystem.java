package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.awt.*;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle m_candle = new CANdle(Constants.LEDConstants.CANdle);
    private final int LedCount = 300;
    private XboxController joystick;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }
    private AnimationTypes m_currentAnimation;


    public LEDSubsystem(XboxController joy){
        this.joystick = joy;
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }


    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, LarsonAnimation.BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinkleAnimation.TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffAnimation.TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(CANdle.LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }


    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Empty); break;
            case Empty: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Empty); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case Empty: changeAnimation(AnimationTypes.TwinkleOff); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }

    public void SetAnimation(Animation animation){
        m_candle.animate(animation);
    }

    public void MakeRed(){
        m_candle.setLEDs(238,75,43);
    }
    public void MakeBlue(){
        float[] color = new float[3];
        Color.RED.getRGBColorComponents(color);
        m_candle.setLEDs((int) color[0], (int) color[1], (int) color[2]);
    }

    public void MakeGreen(){
        m_candle.setLEDs(238,75,43);
    }

    public void MakeYellow(){
        m_candle.setLEDs(238,75,43);
    }

    public void MakePurple(){
        m_candle.setLEDs(238,75,43);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(m_toAnimate == null) {
            m_candle.setLEDs((int)(joystick.getLeftTriggerAxis() * 255),
                    (int)(joystick.getRightTriggerAxis() * 255),
                    (int)(joystick.getLeftX() * 255));
        } else {
            m_candle.animate(m_toAnimate);
        }
        m_candle.modulateVBatOutput(joystick.getRightY());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

