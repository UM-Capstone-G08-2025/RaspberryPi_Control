import serial

# Initialize serial communication
try:
    data = serial.Serial("/dev/ttyS0", 115200, timeout=1)  # Added timeout for better control
    print("Serial connection established.")
except serial.SerialException as e:
    print(f"Error initializing serial connection: {e}")
    exit(1)

# Read data in a loop
try:
    while True:
        try:
            # Read and process data from UART
            data2 = data.readline().decode('utf-8').strip()
            if data2:  # Ensure non-empty data is printed
                print(f"Received: {data2}")
        except UnicodeDecodeError as e:
            print(f"Decoding error: {e}")
except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    # Clean up serial connection
    if data.is_open:
        data.close()
    print("Serial connection closed.")
