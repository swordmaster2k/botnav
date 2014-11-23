def write_paths(stream, paths):

    for path in paths:
        for point in path:
            stream.write('\t' + '%.2f' % point[0] + '\t' '%.2f' % point[1] + '\n')