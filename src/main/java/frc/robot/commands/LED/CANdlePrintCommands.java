package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class CANdlePrintCommands {
    static public class PrintVBat extends InstantCommand {
        public PrintVBat(LEDSubsystem candleSystem) {
            super(() -> System.out.println("Vbat is " + candleSystem.getVbat() + "V"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class Print5V extends InstantCommand {
        public Print5V(LEDSubsystem candleSystem) {
            super(() -> System.out.println("5V is " + candleSystem.get5V() + "V"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class PrintCurrent extends InstantCommand {
        public PrintCurrent(LEDSubsystem candleSystem) {
            super(() -> System.out.println("Current is " + candleSystem.getCurrent() + "A"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class PrintTemperature extends InstantCommand {
        public PrintTemperature(LEDSubsystem candleSystem) {
            super(() -> System.out.println("Temperature is " + candleSystem.getTemperature() + "C"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}