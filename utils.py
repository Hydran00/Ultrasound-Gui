def read_from_file(filename):
    listed = []
    with open(filename,"r") as text:
        Line = text.readline()
        while Line!='':
            listed.append(Line.rstrip("\n"))
            Line = text.readline()

    return listed