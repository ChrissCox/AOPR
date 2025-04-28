import serial
import time

def send_cmd_servo_move(ser, servo_params, move_time):
    """
    Sends the CMD_SERVO_MOVE command to the servo controller.
    
    Parameters:
      ser          : The serial.Serial instance.
      servo_params : A list of tuples [(servo_id, angle_value), ...].
      move_time    : The move time as an integer (e.g., in ms).
    """
    # Number of servos to control.
    num_servos = len(servo_params)
    
    # Calculate the data length.
    # The protocol requires: length = (number of servos * 3) + 5
    data_length = num_servos * 3 + 5
    
    # Command value for CMD_SERVO_MOVE is 3.
    command = 0x03
    
    # Split move_time into lower and higher 8 bits.
    time_low = move_time & 0xFF
    time_high = (move_time >> 8) & 0xFF

    # Construct the parameter list:
    # First three parameters: number of servos, time_low, time_high.
    params = [num_servos, time_low, time_high]
    
    # Append servo-specific parameters.
    for servo_id, angle in servo_params:
        # Split angle into lower and higher 8 bits.
        angle_low = angle & 0xFF
        angle_high = (angle >> 8) & 0xFF
        params.extend([servo_id, angle_low, angle_high])
    
    # Build the full packet:
    # Header (2 bytes) + Data Length + Command + Parameters
    packet = [0x55, 0x55, data_length, command] + params
    
    # Send the packet over serial.
    ser.write(bytearray(packet))
    print("Packet sent:", packet)

def send_data_back(ser,servo_params,move_time):
# Number of servos to control.
    num_servos = len(servo_params)
    
    # Calculate the data length.
    # The protocol requires: length = (number of servos +3)
    data_length = num_servos + 3
    
    # Command value for CMD_SERVO_MOVE is 3.
    command = 0x15
    
    # Split move_time into lower and higher 8 bits.
    time_low = move_time & 0xFF
    time_high = (move_time >> 8) & 0xFF

    # Construct the parameter list:
    # First three parameters: number of servos, time_low, time_high.
    params = [num_servos, time_low, time_high]
    
    # Append servo-specific parameters.
    for servo_id, angle in servo_params:
        # Split angle into lower and higher 8 bits.
        angle_low = angle & 0xFF
        angle_high = (angle >> 8) & 0xFF
        params.extend([servo_id, angle_low, angle_high])
    
    # Build the full packet:
    # Header (2 bytes) + Data Length + Command + Parameters
    packet = [0x55, 0x55, data_length, command] + params
    
    # Send the packet over serial.
    ser.write(bytearray(packet))
    print("Packet received:", packet)


def main():
    # Adjust the serial port to match your setup (e.g., /dev/ttyUSB0).
    serial_port = '/dev/ttyUSB0'
    
    try:
        ser = serial.Serial(serial_port, 9600, timeout=1)
        time.sleep(2)  # Allow time for the serial connection to initialize

        # Define servo commands.
        # Example: control two servos, e.g. servo 1 to angle 500, servo 2 to angle 600.
        servo_commands = [(1,1400),(2,900),(3,900),(4,1500),(5,500),(6,1500)] #,(2, 600),(3,400),(4,400)
        move_time = 3000  # Movement duration in milliseconds (adjust based on your requirements)
        # Send the CMD_SERVO_MOVE command.
        send_cmd_servo_move(ser, servo_commands, move_time)
        time.sleep(3)

        second = [(4,2500)] #,(2, 600),(3,400),(4,400)
        move_time = 3000  # Movement duration in milliseconds (adjust based on your requirements)
        # Send the CMD_SERVO_MOVE command.
        send_cmd_servo_move(ser, second, move_time)
        time.sleep(3)


        close_servo = [(1,2500),(2,900),(3,900),(4,2500),(5,2500)]#[(1,2500),(2,2500),(3,500),(4,500),(5,5000),(6,2500)]
        move_t=5000
        send_cmd_servo_move(ser,close_servo,move_t)
        time.sleep(3)


   


        fourth = [(5,500),(6,1000)]#[(1,2500),(2,2500),(3,500),(4,500),(5,5000),(6,2500)]
        move_t=5000
        send_cmd_servo_move(ser,fourth,move_t)
        time.sleep(3)

        close = [(1,1500)]
        move_time = 5000  # Movement duration in milliseconds (adjust based on your requirements)
        send_cmd_servo_move(ser,close,move_t)

        #send_data_back(ser,close,move_t)

    except serial.SerialException as e:
        print("Error opening serial port:", e)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
