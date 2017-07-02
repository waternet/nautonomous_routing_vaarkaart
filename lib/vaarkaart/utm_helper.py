import utm

# Convert GPS string to UTM
def convert_GPS_string_to_UTM(gps_string):
	gpsPosition = map(float, gps_string.split(",", 1))

	utmPosition = utm.from_latlon(gpsPosition[0], gpsPosition[1])[:2] # only use the first two elements of the tuple: easting and northing (in that order)
	utmString = str(utmPosition[0]) + "," + str(utmPosition[1])

	return utmString, utmPosition[0], utmPosition[1]

# Convert GPS array to UTM
def convert_GPS_array_to_UTM(gps_array):
	result = []

	for gps_string in gps_array:
		result.append(convert_GPS_string_to_UTM(gps_string)[0])

	return result

def map_float_split(value):
	return map(float, value.split(","))