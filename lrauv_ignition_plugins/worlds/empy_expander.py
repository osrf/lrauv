# Takes in a template and generates an SDF file.

import em
import os
import sys

def main(in_file, out_file):
    with open(in_file) as infile:
        template = infile.read()
    result = em.expand(template, {})
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, 'w') as outfile:
        outfile.write(result)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage:")
        print("empy_expander.py <infile> <outfile>")
        exit(-100)

    sys.exit(main(sys.argv[1], sys.argv[2]))
