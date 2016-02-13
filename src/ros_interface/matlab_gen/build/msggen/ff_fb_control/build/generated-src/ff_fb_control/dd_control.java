package ff_fb_control;

public interface dd_control extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ff_fb_control/dd_control";
  static final java.lang.String _DEFINITION = "Header header\nfloat64 desired_x\nfloat64 desired_y\nfloat64 desired_theta\nfloat64 desired_linear_velocity\nfloat64 desired_angular_velocity";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getDesiredX();
  void setDesiredX(double value);
  double getDesiredY();
  void setDesiredY(double value);
  double getDesiredTheta();
  void setDesiredTheta(double value);
  double getDesiredLinearVelocity();
  void setDesiredLinearVelocity(double value);
  double getDesiredAngularVelocity();
  void setDesiredAngularVelocity(double value);
}
