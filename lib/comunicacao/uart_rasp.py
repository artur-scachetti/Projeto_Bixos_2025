import serial
import struct
import time
import keyboard

try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0)

except serial.SerialException as e:

    print(f"Erro ao abrir porta serial: {e}")
    exit()


# def crc_calc(data : bytes) -> int:

#     crc = 0
    
#     for b in data:
    
#         crc ^= b

#     return crc


# def receive_data():

#     buffer_size = 9

#     while True:
#         try:
#             if ser.in_waiting >= buffer_size:

#                 data = ser.read(buffer_size)

#                 data_bytes = data[:-1]
#                 crc_rec = data[-1]

#                 crc_val = crc_calc(data_bytes)

#                 if crc_val == crc_rec:

#                     left_rads, right_rads = struct.unpack('<ff', data)
#                     print(f"Left Rad/s: {left_rads} | Right Rad/s: {right_rads}")

#                 else:

#                     print("Erro: CRC inválido")

#             else:

#                 time.sleep(0.01)    
            
#         except (ValueError, serial.SerialException) as e:
            
#             print(f"Erro ao processar dados ou na comunicação serial: {e}")


def teste_uart_rasp():

#   teste 1 - manda float

    data = 10

    data_bytes = struct.pack("<f", data)

    ser.write(data_bytes)

    ser.close

#     teste 2 - teclado (vel infinita)

#     while True:

#         if keyboard.is_pressed("Up"):

#             ser.write(b'U')
              
#             ser.close()
#             break



# if __name__ == "__main__":

#     print("Mandando dados para ESP...")
#     teste_uart_rasp()
