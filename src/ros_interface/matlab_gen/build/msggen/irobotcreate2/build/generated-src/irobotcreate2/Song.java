package irobotcreate2;

public interface Song extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Song";
  static final java.lang.String _DEFINITION = "Header header\nuint8 song_number\nNote[] notes\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getSongNumber();
  void setSongNumber(byte value);
  java.util.List<irobotcreate2.Note> getNotes();
  void setNotes(java.util.List<irobotcreate2.Note> value);
}
