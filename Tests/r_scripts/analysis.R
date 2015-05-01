############################## Analysis of Grid Nav ######################################
hist(gridnav$traverse)

# Get worst case scenario
val = max(gridnav$traverse)
index = which(gridnav$traverse == val)
gridnav[index, ]

gridnav_accesses_mean = mean(gridnav$accesses)
median(gridnav$accesses)

gridnav_traverse_mean = mean(gridnav$traverse)
median(gridnav$traverse)

gridnav_total_mean = gridnav_accesses_mean + gridnav_traverse_mean

############################## Analysis of D* Lite #######################################
hist(d_star_lite$traverse)

val = max(d_star_lite$traverse)
index = which(d_star_lite$traverse == val)
d_star_lite[index, ]

d_star_lite_accesses_mean = mean(d_star_lite$accesses)
median(d_star_lite$accesses)

d_star_lite_traverse_mean = mean(d_star_lite$traverse)
median(d_star_lite$traverse)

d_star_lite_total_mean = d_star_lite_accesses_mean + d_star_lite_traverse_mean

############################## Analysis of Theta* ########################################
hist(theta_star$traverse)

val = max(theta_star$traverse)
index = which(theta_star$traverse == val)
theta_star[index, ]

theta_star_accesses_mean = mean(theta_star$accesses)
median(theta_star$accesses)

theta_star_traverse_mean = mean(theta_star$traverse)
median(theta_star$traverse)

theta_star_total_mean = theta_star_accesses_mean + theta_star_traverse_mean

############################## Analysis of Field D * #####################################
hist(field_d_star$traverse)

val = max(field_d_star$traverse)
index = which(field_d_star$traverse == val)
field_d_star[index, ]

field_d_star_accesses_mean = mean(field_d_star$accesses)
median(field_d_star$accesses)

field_d_star_traverse_mean = mean(field_d_star$traverse)
median(field_d_star$traverse)

field_d_star_total_mean = field_d_star_accesses_mean + field_d_star_traverse_mean

############################## Algorithm Comparison ######################################

# Calculate how much more efficient planners are than traditional D* Lite
# heading restrictions.
100 - (gridnav_total_mean / (d_star_lite_total_mean / 100))
100 - (theta_star_total_mean / (d_star_lite_total_mean / 100))
100 - (field_d_star_total_mean / (d_star_lite_total_mean / 100))