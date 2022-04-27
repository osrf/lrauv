# Takes in a template and generates a file.

import argparse
import em
import os
import sys

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', help='Full path to input template.')
    parser.add_argument('outfile', help='Full path to output file.')
    args = parser.parse_args()

    with open(args.infile) as infile:
        template = infile.read()
    result = em.expand(template, {})
    os.makedirs(os.path.dirname(args.outfile), exist_ok=True)
    with open(args.outfile, 'w') as outfile:
        outfile.write(result)

if __name__ == '__main__':
    sys.exit(main())
