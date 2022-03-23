import em
import os
import sys

def main():
    template = open('portuguese_ledge.sdf.em').read()
    arguments = {}
    result = em.expand(template, arguments)
    with open(os.path.join('portuguese_ledge.sdf'), 'w') as outfile:
        outfile.write(result)

if __name__ == '__main__':
    sys.exit(main())
