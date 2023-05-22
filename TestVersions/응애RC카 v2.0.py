import serial
import time
from pynput import keyboard

currentPressed = set()
print("Start")
# print(type(currentPressed))
port="COM3" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput()
result = 's'
mode = False
def on_key_release(key):
   global result
   if result !='s' and mode == False:
      result = 's'
      print(result)
      result_bytes = result.encode('utf_8')
      bluetooth.write(result_bytes)

def on_key_pressed(key):
   global result
   result1 ='%s' % key
   currentPressed.add(key)
   if result == 's':
      if result1=='Key.up' :
         result = 'f'
         print(result)
      elif result1=='Key.down' :
         result = 'b'
         mode = True
         print(result)
      elif result1=='Key.left' :
         result = 'l'
         print(result)
      elif result1=='Key.right' :
         result = 'r'
         print(result)
      result_bytes = result.encode('utf_8')
      bluetooth.write(result_bytes)

with keyboard.Listener(on_release = on_key_release,on_press=on_key_pressed) as listener:
   listener.join()