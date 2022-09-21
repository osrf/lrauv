#!/usr/bin/python3
import pandas as pd
import sys

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print ("Usage: invert_csv_depth.py <file_to_be_inverted>")
    data = pd.read_csv(sys.argv[1])
    data["depth_meter"] = -data["depth_meter"]
    data.to_csv(sys.argv[1])