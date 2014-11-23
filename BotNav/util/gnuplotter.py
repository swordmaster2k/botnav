def write_paths(stream, paths):
    i = 0
    j = 0
    longest = 0
    loops = 0

    for path in paths:
        if len(path) > longest:
            longest = len(path)

    # Make this more efficient!
    while j < longest - 1:
        if i == len(paths):
            i = 0
            j += 1
            stream.write('\n')

        if len(paths[i]) > j:
            stream.write('\t' * j)
            stream.write('%.2f' % paths[i][j][0] + '\t' '%.2f' % paths[i][j][1])
            stream.write('\t' * (j + 1))

            # If we have dealt with all of the points from
            # this path remove it from the paths list.
            if len(paths[i]) == j + 1:
                paths.pop(i)
                i -= 1

        i += 1
        loops += 1

    print("\nloops: " + str(loops) + "\n")

import sys


def test_write_paths():
    path1 = [(10, 10)]
    path2 = [(5, 5), (10, 10)]
    path3 = [(3, 3), (6, 6), (10, 10)]
    path4 = [(4, 4), (10, 10)]
    path5 = [(10, 10)]

    paths = [path1, path2, path3, path4, path5]

    write_paths(sys.stdout, paths)

    path1 = [(3, 3), (6, 6), (10, 10)]
    path2 = [(4, 4), (10, 10)]
    path3 = [(10, 10)]
    path4 = []

    paths = [path1, path2, path3, path4]

    write_paths(sys.stdout, paths)

test_write_paths()