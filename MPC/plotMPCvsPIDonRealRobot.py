
import matplotlib.pyplot as plt
import json

from TestOnRobot import Logger

def readLogg(title: str) -> Logger:
    # This function is made by ChatGPT

    """Load Logger from JSON file."""
    filename = f"MPC/DATA/{title}.json"
    with open(filename, "r") as f:
        data = json.load(f)
    return Logger(**data)

def compare_logs(title1: str, title2: str):
    # This function is made by ChatGPT

    # Read logs
    log1 = readLogg(title1)
    log2 = readLogg(title2)

    # --- Time alignment: subtract the first timestamp ---
    t1 = [t - log1.time[0] for t in log1.time]
    t2 = [t - log2.time[0] for t in log2.time]

    # Create figure
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Angle
    axs[0].plot(t1, log1.angle0, label=title1)
    axs[0].plot(t2, log2.angle0, label=title2)
    axs[0].set_ylabel("Angle")
    axs[0].legend()

    # Velocity
    axs[1].plot(t1, log1.vel0, label=title1)
    axs[1].plot(t2, log2.vel0, label=title2)
    axs[1].set_ylabel("Velocity")

    # Power
    axs[2].plot(t1, log1.power0, label=title1)
    axs[2].plot(t2, log2.power0, label=title2)
    axs[2].set_ylabel("Power")
    axs[2].set_xlabel("Time (s)")

    plt.suptitle(f"Comparison: {title1} vs {title2}")
    plt.tight_layout()
    plt.show()

compare_logs("PIDLogg", "MPCLogg")