a = True

file_name = "SensorData"

csv_file = open(file_name+'.csv','a')
print('Complete loading ' + str(file_name)+'.csv')

binary_file = open(file_name+'.bin','rb')
print('Complete loading ' + str(file_name)+'.bin')

print("")

print('Converting...')
csv_file.write('No,Temperature,Pressure,accelX,accelY,accelZ,magX,magY,magZ,gyroX,gyroY,gyroZ,gps_latitude,gps_longtitude,gps_time,highte,accel\n')

n = 1

sensor = [0]*14

while a:


  for i in range(14):
    binary_data = binary_file.read(8)

    if len(binary_data) == 0:
      a = False
      
    sensorValue = int.from_bytes(binary_data,byteorder='big',signed=True)
      

    if 0<= i <=10:
      sensor[i] = sensorValue/1000
    elif 11 <= i <= 12:
      sensor[i] = sensorValue/1000000000
    else:
      sensor[i] = sensorValue

  if a == False:
    break
  
  csv_file.write(str(n))
  csv_file.write(',')   

  for i in range(14):
    csv_file.write(str(sensor[i]))
    if 0 <= i <=13:
      csv_file.write(',')
     

  t = sensor[0]
  p = sensor[1]
  x = sensor[2]
  y = sensor[3]
  z = sensor[4]

  highte = (((101325/p)**(1/5.257)-1))*(t+273.15)/0.0065
  accel = (x**2+y**2+z**2)**1/2

  csv_file.write(str(highte))
  csv_file.write(',')
  csv_file.write(str(accel))
  csv_file.write('\n')

  print('Complete: ' + str(n))
  n += 1
  
print('Done!')

csv_file.close()
