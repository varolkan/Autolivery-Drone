# Obstacles reported by one source (ADS-B, a perception stack, simulated traffic).
# Every message replaces the previous one from the same source; a source that stops
# publishing has its obstacles dropped after the mission node's obstacle timeout.
std_msgs/Header header
string source
autolivery_interfaces/Obstacle[] obstacles
