def write_paths(stream, paths):
    i = 0
    j = 0
    longest = 0
    paths_length = len(paths)

    #loops = 0 # For debugging.

    for path in paths:
        if len(path) > longest:
            longest = len(path)

    while j < longest:
        if len(paths[i]) > j:
            if paths[i][j][0] < 10:
                point1 = '%.3f' % paths[i][j][0]
            else:
                point1 = '%.2f' % paths[i][j][0]

            if paths[i][j][1] < 10:
                point2 = '%.3f' % paths[i][j][1]
            else:
                point2 = '%.2f' % paths[i][j][1]

            # Character width is 20.
            stream.write(point1 + "     " + point2 + "     ")

            # If we have dealt with all of the points from
            # this path remove it from the paths list.
            #if len(paths[i]) == j + 1:
            #    paths.pop(i)
            #    i -= 1
        else:
            # Character width is 20 so write the blank columns.
            stream.write(" " * 20)

        i += 1

        if i == paths_length:
            i = 0
            j += 1
            stream.write('\n')

        #loops += 1

    #print("\nloops: " + str(loops) + "\n")

import os


def generate_output(directory, paths, grid_size, output_type):
    gnuplot = os.popen('/usr/bin/gnuplot', 'w')

    column1 = 1
    column2 = 2

    if not gnuplot.closed:
        configuration = "set terminal " + str(output_type) + ";" + "set xlabel \'Grid Width (Cells)\'; set ylabel " \
                                                                   "\'Grid Height (Cells)\';"
        gnuplot.write(configuration)

        for i in range(paths):
            # Setup the output for gnuplot.
            configuration = "set output '" + directory + "/path" + str(i) + ".png'; set title \"Path " + str(i) + "\";"

            # First command for plot contains the robots path.
            commands1 = "plot [0:" + str(grid_size) + "] [0:" + str(grid_size) + "] " + '\"' + directory + \
                        '/paths.gnuplot\"' + " using " + str(column1) + ":" + str(column2) + \
                        " title \'path\' with linespoints"

            # Second command contains the obstacles in the environment.
            commands2 = '\"' + directory + '/occupancy.gnuplot\"' + " using " + str(1) + ":" + str(2) + \
                        " title \'obstacles\' with points\n"

            gnuplot.write(configuration)
            gnuplot.write(commands1 + "," + commands2)

            column1 += 2
            column2 += 2

        gnuplot.close()
    else:
        print("gnuplot not available please install it.")

import sys


def test_write_paths():
    path1 = [(10, 10)]
    path2 = [(5, 5), (10, 10)]
    path3 = [(3, 3), (6, 6), (10, 10)]
    path4 = [(4, 4), (10, 10)]
    path5 = [(10, 10)]

    paths = [path1, path2, path3, path4, path5]

    write_paths(sys.stdout, paths)
    sys.stdout.write('\n')

    path1 = [(3, 3), (6, 6), (10, 10)]
    path2 = [(4, 4), (10, 10)]
    path3 = [(10, 10)]

    paths = [path1, path2, path3]

    write_paths(sys.stdout, paths)
    sys.stdout.write('\n')

    path1 = [(5, 5), (10, 10)]
    path2 = [(10, 10)]
    path3 = [(3, 3), (6, 6), (10, 10)]
    path4 = [(10, 10)]
    path5 = [(4, 4), (10, 10)]

    paths = [path1, path2, path3, path4, path5]

    write_paths(sys.stdout, paths)

#test_write_paths()