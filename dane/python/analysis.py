import pandas as pd
import numpy as np
from collections import deque

# Ścieżki
input_file = "filt1_25hz.xlsx"
output_file = "statistics_800hz.xlsx"

# Wczytaj dane z Excela
df = pd.read_excel(input_file)

# Bufory FIFO (po 10 próbek) dla każdej kolumny z danymi (czyli wszystkie poza 't')
data_columns = df.columns[1:]
buffers = {col: deque(maxlen=5) for col in data_columns}

# Lista na wynikowe wiersze
results = []

# Iteracja po wierszach
for idx, row in df.iterrows():
    time = row["t"]
    result_row = {"t": time}

    for col in data_columns:
        val = row[col]
        buffers[col].append(val)

        if len(buffers[col]) == 5:
            buf = np.array(buffers[col], dtype=np.float64)
            result_row[f"mean_{col}"] = float(np.mean(buf))
            result_row[f"stdev_{col}"] = float(np.std(buf, ddof=1))
            result_row[f"median_{col}"] = float(np.median(buf))
            result_row[f"diff_{col}"] = float(np.max(buf) - np.min(buf))
        else:
            result_row[f"mean_{col}"] = None
            result_row[f"stdev_{col}"] = None
            result_row[f"median_{col}"] = None
            result_row[f"diff_{col}"] = None

    results.append(result_row)

# Zapisz do pliku Excel
results_df = pd.DataFrame(results)
results_df.to_excel(output_file, index=False)

print(f"Zapisano wynik: {output_file}")