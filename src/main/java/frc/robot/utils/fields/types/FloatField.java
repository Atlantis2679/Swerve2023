package frc.robot.utils.fields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.robot.utils.fields.FieldBase;

public class FloatField extends FieldBase<Float> {
    public FloatField(String key, Supplier<Float> valueSupplier, float defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getFloat(key, value);
    }
}
