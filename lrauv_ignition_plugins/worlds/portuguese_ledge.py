# Takes in a template and generates an SDF file.

import em
import os
import sys

def main():
    in_file = 'portuguese_ledge.sdf.em'
    out_file = 'portuguese_ledge.sdf'
    template = open(in_file).read()
    arguments = {}
    result = em.expand(template, arguments)
    with open(os.path.join(out_file), 'w') as outfile:
        outfile.write(result)

if __name__ == '__main__':
    sys.exit(main())
