import pandas as pd
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/tmp/planner_metrics.csv", header=None)

df.columns = ["planning_time_ms", "num_poses", "path_length"]

print(df)


plt.plot(df["planning_time_ms"])
plt.title("Planning time (ms)")
plt.show()
