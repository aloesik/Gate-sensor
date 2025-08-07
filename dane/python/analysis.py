import pandas as pd
import numpy as np
from collections import deque

# Ścieżki
input_file = "big_full.xlsx"
output_file = "statistics_800hz.xlsx"

# Wczytaj dane z Excela
df = pd.read_excel(input_file)

# Bufory FIFO (po 10 próbek) dla każdej kolumny z danymi (czyli wszystkie poza 't')
data_columns = df.columns[1:]
buffers = {col: deque(maxlen=10) for col in data_columns}

# Lista na wynikowe wiersze
results = []

# Iteracja po wierszach
for idx, row in df.iterrows():
    time = row["t"]
    result_row = {"t": time}

    for col in data_columns:
        val = row[col]
        buffers[col].append(val)

        if len(buffers[col]) == 10:
            buf = np.array(buffers[col], dtype=np.float64)
            result_row[f"{col}_mean"] = float(np.mean(buf))
            result_row[f"{col}_stdev"] = float(np.std(buf, ddof=1))
            result_row[f"{col}_median"] = float(np.median(buf))
            result_row[f"{col}_diff"] = float(np.max(buf) - np.min(buf))
        else:
            result_row[f"{col}_mean"] = None
            result_row[f"{col}_stdev"] = None
            result_row[f"{col}_median"] = None
            result_row[f"{col}_diff"] = None

    results.append(result_row)

# Zapisz do pliku Excel
results_df = pd.DataFrame(results)
results_df.to_excel(output_file, index=False)

print(f"Zapisano wynik: {output_file}")