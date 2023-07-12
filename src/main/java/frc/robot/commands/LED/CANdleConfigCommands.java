package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class CANdleConfigCommands {
    static public class ConfigBrightness extends InstantCommand {
        public ConfigBrightness(LEDSubsystem candleSystem, double brightnessPercent) {
            super(() -> candleSystem.configBrightness(brightnessPercent), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class ConfigLosBehavior extends InstantCommand {
        public ConfigLosBehavior(LEDSubsystem candleSystem, boolean disableWhenLos) {
            super(() -> candleSystem.configLos(disableWhenLos), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class ConfigStatusLedBehavior extends InstantCommand {
        public ConfigStatusLedBehavior(LEDSubsystem candleSystem, boolean disableWhile) {
            super(() -> candleSystem.configStatusLedBehavior(disableWhile), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}
