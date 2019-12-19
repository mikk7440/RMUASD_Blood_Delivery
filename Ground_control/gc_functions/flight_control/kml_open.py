#!/usr/bin/python2
import re
import time
# import urllib
# Note to use other version
from urllib.request import urlretrieve
import matplotlib.pyplot as plt
from pykml import parser


polygons = []
polygons_lat=[]
polygons_long=[]
lat=[]
lon=[]
temp_short_x = []
temp_short_y = []

#clear the lists
def ClearLists():
    #print "Clear the lists"
    del polygons[:]
    del polygons_lat[:]
    del polygons_long[:]
    del lat[:]
    del lon[:]
    del temp_short_x[:]
    del temp_short_y[:]

# Download the kml file
def Download_KML():
    url = 'https://droneid.dk/rmuasd/rmuasd_nofly.kml'
    # urllib.urlretrieve(url, "rmuasd_nofly.kml")
    urlretrieve(url, "rmuasd_nofly.kml")

# Get coordinates from kml file to a txt file
def get_coordinates():
    kml_string = open("rmuasd_nofly.kml", "r").read()
    root = parser.fromstring(kml_string.encode('utf-8'))
    file = open("coor.txt", 'w+')
    try:
        count = 0
        counter = 1
        while count < 10:  # This assumes there are less than 10 no fly zones
            # print counter
            # Read the coordinates from the file
            coords = root.Document.Placemark[count].Polygon.outerBoundaryIs.LinearRing.coordinates
            file.write(str(coords))
            count += 1
            counter += 1
        else:
            print("Max count value reached %d, if you are getting this message increate XX in while(count<XX)" %counter)
    except:
        # This bit gets executed when the program throws an error
        # print "Error caught"
        counter -= 1  # R educe it by one to get the real number of objects on the map
        # print "There are", counter,"areas to avoid"
        # print "Close file due to error catch"
        file.close()
    file.close()


# read the txt file with coordinates
def getlatlon():

    Download_KML()
    get_coordinates()

    poly = 0
    return_val = []
    coords = []
    with open("coor.txt", "r") as f:
        for cnt, line in enumerate(f):
            parts = line.split(',')
            # print(line)
            # If len(parts) Is bigger than 2 it is an object
            if len(parts) > 2:
                # print("Append to coords")
                coords.append([float(parts[1]) * 100000, float(parts[0]) * 100000])
                lat.append(float(parts[1]))
                lon.append(float(parts[0]))
            elif len(lat) > 0 and len(lon) > 0:
                # print "Line was short, line:", cnt
                # print poly
                poly += 1
                polygons_lat.append(lat)
                polygons_long.append(lon)
                if len(coords) > 0:
                    coords.pop()
                    return_val.append(coords)
                coords = []
                # print("Append to return")

    if len(coords) > 0:
        coords.pop()
        return_val.append(coords)
    #print "\nReturn values:", len(return_val), "\n", return_val
    return return_val


# Check if file is empty and if not delete the content
def checkfile():
    with open("coor.txt", "w+") as f:
        # print("file not empty")
        f.write('')


#Plot the obstacles
def plotShape():
    # print "Now to plot"
    # #print polygons_lat
    for i in range(len(polygons_lat[0])):
        temp_short_x.append(polygons_lat[0][i])
        temp_short_y.append(polygons_long[0][i])
        plt.plot(temp_short_x, temp_short_y, color='b')

    plt.xlabel('lat')
    plt.ylabel('lon')

    plt.show(block=False)
    # print "Showing the plot for 3 seconds"
    plt.pause(3)
    plt.close()


if __name__ == "__main__":
    old = getlatlon()
    while True:

        # Download_KML()
        # get_coordinates()
        new = getlatlon()
        print(old == new)
        old = new
        # plotShape()
        # checkfile()

