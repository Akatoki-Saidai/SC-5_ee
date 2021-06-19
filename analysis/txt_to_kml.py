files = open("")
data = files.readlines()

for i in data:
    if i.find("LAT"):
        LAT = float(i[22:])
    elif i.find("LONG"):
