# Readable message that describes the error
std_msgs/Header header
string error_message

# The type of the error, the type is used to determine how to handle the error
uint16 error_code

# The severity of the error, used to determine how to handle the error
uint8 severity