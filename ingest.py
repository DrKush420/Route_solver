import pandas as pd

df = pd.read_csv("history.csv", parse_dates=["Submission Date"])
pd.set_option("display.max_columns", None)  # Show all columns
pd.set_option("display.expand_frame_repr", False)
print(df.columns.tolist())
df["Submission Date"] = pd.to_datetime(df["Submission Date"], format="%d/%m/%Y")
df["START UUR "] = pd.to_datetime(df["START UUR "], format="%H:%M").dt.time

df_sorted = df.sort_values(by=["Submission Date", "START UUR "], ascending=[True, True])

df.head(100).to_csv("output.csv", index=False)


print(df.head(100))
