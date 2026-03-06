#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt


def main(f):
    df = pd.read_csv(f)
    print(df.info())
    df = df.set_index(df['time'])
    df[['bat.vp']].plot()
    plt.show()


if __name__ == '__main__':
    main("./batteryModel_res.csv")
