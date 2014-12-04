import dstarlite

# open sample map
dstarlite.setup("map.txt")

# do initial plan
print("initial plan")
dstarlite.plan()
print(dstarlite.getvertexaccesses())
print(dstarlite.getrobotpath())
print(dstarlite.getoccupancygrid())
print(dstarlite.getcostgrid())

'''
# move the robot somewhere else
#
# order here is significant robot must
# be moved before any updates to the
# occupancy grid
print("move robot")
dstarlite.updaterobotposition(1, 7)

# simulate scan and occupy some cells
print("occupy some cells")
#dstarlite.updatecelloccupancy(4, 7, 1)
#dstarlite.updatecelloccupancy(4, 6, 1)
dstarlite.updatecelloccupancy(4, 5, 1)

# compute the new path
print("compute new path")
print(dstarlite.replan())
dstarlite.printpath()
dstarlite.printoccupancygrid()

print("move robot")
dstarlite.updaterobotposition(5, 2)

# compute the new path
print("compute new path")
print(dstarlite.replan())
dstarlite.printpath()
dstarlite.printoccupancygrid()
'''
