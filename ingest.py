import re
import pandas as pd
import psycopg2


def get_stop_time(row):
    if re.search(r'ZERE: 3-4 zit', row['type_reiniging']):
        return 90
    elif re.search(r'ZERE: 5-6 zit', row['type_reiniging']):
        return 120
    elif re.search(r'ZERE: 7-8 zit', row['type_reiniging']):
        return 180
    else:
        return None


def get_address_stop_time(df, x):
    result = df[['locatie', 'stop_time']].dropna(subset=['stop_time']).head(x)
    return list(result.itertuples(index=False, name=None))


def load_and_process_dataframe():
    # PostgreSQL connection parameters
    conn = psycopg2.connect(
        host="routes-db.cpwqg8mayvig.eu-north-1.rds.amazonaws.com",
        dbname="postgres",
        user="postgres",
        password="deepmaind",
        port=5432
    )

    # Load data from a table (replace 'your_table_name' with actual table name)
    query = "SELECT * FROM history;"  # Replace accordingly
    df = pd.read_sql(query, conn)
    df = df.applymap(lambda x: x.encode('latin1').decode('utf-8') if isinstance(x, str) else x)

    conn.close()

    df["submission_date"] = pd.to_datetime(df["submission_date"], format="%d/%m/%Y")
    df.rename(columns=lambda x: x.strip(), inplace=True)
    df["start_uur"] = pd.to_datetime(df["start_uur"], errors="coerce").dt.time
    df["einduur"] = pd.to_datetime(df["einduur"], errors="coerce").dt.time

    df_sorted = df.sort_values(by=["submission_date", "start_uur"], ascending=[True, True])
    df_sorted['stop_time'] = df_sorted.apply(get_stop_time, axis=1)

    return get_address_stop_time(df_sorted, 18)

