% euler distance
function h = util_EulerDist(x_start,y_start,x_destination, y_destination)

h = sqrt((x_start-x_destination)^2+(y_start-y_destination)^2);