fr = open('nose_data.bin', 'rb')
fw = open('test.csv', 'wb')

while True:
  data = fr.read(1)
  if len(data) == 0:
    break
  fw.write(data)

fw.close()
fr.close()
