import re
import pandas as pd


def get_stop_time(row):
    if re.search(r'ZERE: 3-4 zit', row['TYPE REINIGING']):
        return 90
    elif re.search(r'ZERE: 5-6 zit', row['TYPE REINIGING']):
        return 120
    elif re.search(r'ZERE: 7-8 zit', row['TYPE REINIGING']):
        return 180
    else:
        return None


def get_address_stop_time(df, x):
    # Select the first 'x' rows from the DataFrame with relevant columns
    result = df[['LOCATIE', 'stop_time']].dropna(subset=['stop_time']).head(x)

    # Convert the selected rows into a list of tuples
    address_stop_time_tuples = list(result.itertuples(index=False, name=None))

    return address_stop_time_tuples


def load_and_process_dataframe(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path, parse_dates=["Submission Date"])

    # Convert columns to appropriate types
    df["Submission Date"] = pd.to_datetime(df["Submission Date"], format="%d/%m/%Y")
    df.rename(columns=lambda x: x.strip(), inplace=True)  # Fix column names with extra spaces
    df["START UUR"] = pd.to_datetime(df["START UUR"], format="%H:%M").dt.time
    df["EINDUUR"] = pd.to_datetime(df["EINDUUR"], format="%H:%M").dt.time

    # Sort the dataframe
    df_sorted = df.sort_values(by=["Submission Date", "START UUR"], ascending=[True, True])

    # Add stop_time column
    df_sorted['stop_time'] = df_sorted.apply(get_stop_time, axis=1)
    return get_address_stop_time(df_sorted, 18)