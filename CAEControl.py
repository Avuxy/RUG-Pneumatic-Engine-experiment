import serial
import json
import sys
ser =serial.serial('COM4',19200,timeout=1)

while True:
    if ser.in_waiting > 0: #d
        line = ser.readline().decode(' utf-8').rstrip() #creates string, utf-8 is package type, rstrip removes unneccesary spaces
    print(line)
    try:
        jsonObj = json.loads(line)  #create json object
        pot = jsonObj.get('value')  #obtains value from json file
        print("value: ", pot)       #prints value
        ser.reset_input_bugger()    #resets buffer so it can receive new data
    except:
        print("Error")              #shows error if error occur in try loop