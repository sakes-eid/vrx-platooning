import pandas as pd
from pathlib import Path
import sys


def convert_path(input_file):

    input_path = Path(input_file)

    # output filename
    output_path = input_path.parent / (
        input_path.stem + "_local.csv"
    )

    # read csv
    df = pd.read_csv(input_path)

    # first point as origin
    x0 = df.iloc[0]["x"]
    y0 = df.iloc[0]["y"]

    # translate
    df["x"] = df["x"] - x0
    df["y"] = df["y"] - y0

    # save
    df.to_csv(output_path, index=False)

    print("Original origin:")
    print(f"x0={x0}, y0={y0}")

    print("Saved:")
    print(output_path)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(
            "Usage: python path_converter.py xxx.csv"
        )
        exit()

    convert_path(sys.argv[1])
    