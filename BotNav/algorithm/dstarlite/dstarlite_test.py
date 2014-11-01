import dstarlite

# open sample map
dstarlite.setup("map.txt")

# do initial plan
dstarlite.replan()
dstarlite.printoccupancygrid()

# simulate scan and occupy some cells
dstarlite.updatecelloccupancy(4, 7, 1)
dstarlite.updatecelloccupancy(4, 6, 1)
dstarlite.updatecelloccupancy(4, 5, 1)
dstarlite.updatecelloccupancy(1, 3, 1)

# move the robot somewhere else
dstarlite.updaterobotposition(4, 4)

# compute the new path
dstarlite.replan()
dstarlite.printoccupancygrid()
