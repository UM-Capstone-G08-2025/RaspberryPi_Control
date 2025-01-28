import serial

# Initialize serial communication
try:
    data = serial.Serial("/dev/ttyS0", 115200, timeout=1)  # Added timeout for better control
    print("Serial connection established.")
except serial.SerialException as e:
    print(f"Error initializing serial connection: {e}")
    exit(1)

def read_data():
    """
    Read class and bounding box coordinates from UART.

    Returns:
        tuple: (class, x, y, width, height) if data is valid, otherwise None.
    """
    try:
        data_line = data.readline().decode('utf-8').strip()
        if "detected:" in data_line and "at" in data_line:
            # Extract the class
            class_part = data_line.split("detected:")[1].split("at")[0].strip()
            # Extract the coordinates
            start = data_line.find('(') + 1
            end = data_line.find(')')
            coordinates = data_line[start:end].split(',')
            coordinates = list(map(int, coordinates))  # Convert coordinates to integers
            if len(coordinates) == 4:
                return (class_part, *coordinates)
    except (UnicodeDecodeError, ValueError) as e:
        print(f"Error reading data: {e}")
    return None

if __name__ == "__main__":
    try:
        while True:
            data_output = read_data()
            if data_output:
                obj_class, x, y, width, height = data_output
                print(f"Class: {obj_class}, Bounding box: ({x}, {y}, {width}, {height})")
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if data.is_open:
            data.close()
        print("Serial connection closed.")
