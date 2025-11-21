import pandas as pd

CSV_FILE = "wrist_data_1.csv"

def load_data(filename=CSV_FILE):
    df = pd.read_csv(filename, header=None, names=["time_ms", "position_deg", "gain", "movement_description"])

    df["time_s"] = df["time_ms"] / 1000.0

    timestamps = df["time_s"].values
    positions = df["position_deg"].values
    gains = df["gain"].values

    return timestamps, positions, gains, df


if __name__ == "__main__":
    timestamps, positions, gains, df = load_data()
    print("First 10 rows of the CSV data:")
    print(df.head(10))
