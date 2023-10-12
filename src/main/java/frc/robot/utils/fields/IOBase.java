package frc.robot.utils.fields;

public abstract class IOBase {
    protected final FieldsTable fields;

    protected IOBase(FieldsTable fieldsTable){
        fields = fieldsTable;
    }

    protected IOBase(String name){
        fields = new FieldsTable(name);
    }

    public FieldsTable getFieldsTable() {
        return fields;
    }
}