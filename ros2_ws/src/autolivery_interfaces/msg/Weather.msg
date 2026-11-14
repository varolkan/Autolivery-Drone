# Weather at the depot or along the route, from a weather station, a forecast feed or the
# operator. NaN in a field means "not reported"; the mission judges only what it is given.
std_msgs/Header header

# Mean wind: speed in m/s and the direction it blows FROM, in degrees clockwise from true
# north (the meteorological convention: 270 is a westerly, blowing towards the east).
float32 wind_speed
float32 wind_direction
# Strongest gust, m/s.
float32 wind_gust
# Rain or snow, mm/h (liquid equivalent).
float32 precipitation
# Air temperature, degrees Celsius.
float32 temperature
