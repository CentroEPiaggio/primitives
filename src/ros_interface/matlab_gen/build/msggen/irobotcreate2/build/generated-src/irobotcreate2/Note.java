package irobotcreate2;

public interface Note extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Note";
  static final java.lang.String _DEFINITION = "uint8 note\nuint8 length\n";
  byte getNote();
  void setNote(byte value);
  byte getLength();
  void setLength(byte value);
}
