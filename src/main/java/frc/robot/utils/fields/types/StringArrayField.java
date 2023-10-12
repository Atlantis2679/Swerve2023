package frc.robot.utils.fields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.robot.utils.fields.FieldBase;

public class StringArrayField extends FieldBase<String[]> {
    public StringArrayField(String key, Supplier<String[]> valueSupplier, String[] defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getStringArray(key, value);
    }
}
