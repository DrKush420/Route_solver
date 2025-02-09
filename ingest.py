import pandas as pd
from datetime import date


def process_csv(file_path):
    # Load CSV and parse dates
    df = pd.read_csv(file_path, parse_dates=["Submission Date"])

    # Convert columns to appropriate types
    df["Submission Date"] = pd.to_datetime(df["Submission Date"], format="%d/%m/%Y")  # Fix format here
    df.rename(columns=lambda x: x.strip(), inplace=True)  # Fix column names with extra spaces
    df["START UUR"] = pd.to_datetime(df["START UUR"], format="%H:%M").dt.time
    df["EINDUUR"] = pd.to_datetime(df["EINDUUR"], format="%H:%M").dt.time

    # Sort the dataframe
    df_sorted = df.sort_values(by=["Submission Date", "START UUR"], ascending=[True, True])

    # Initialize lists for alternating row distribution
    df_a_list, df_b_list = [], []

    # Group by Submission Date and distribute alternating rows
    for _, group in df_sorted.groupby("Submission Date"):
        group = group.reset_index(drop=True)
        df_a_list.append(group.iloc[::2])  # Odd-indexed rows
        df_b_list.append(group.iloc[1::2])  # Even-indexed rows

    # Create final DataFrames
    df_a = pd.concat(df_a_list).reset_index(drop=True)
    df_b = pd.concat(df_b_list).reset_index(drop=True)

    # Convert to dictionary with date keys
    locatie_map_a = {key.date(): value for key, value in
                     df_a.groupby("Submission Date")["LOCATIE"].apply(list).to_dict().items()}
    locatie_map_b = {key.date(): value for key, value in
                     df_b.groupby("Submission Date")["LOCATIE"].apply(list).to_dict().items()}

    return locatie_map_a, locatie_map_b


# Example usage
file_path = "history.csv"
locatie_map_a, locatie_map_b = process_csv(file_path)

# Print results
print("LOCATIE Map for DataFrame A:", locatie_map_a)
print("LOCATIE Map for DataFrame B:", locatie_map_b)
