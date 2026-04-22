import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

TABLE_LENGTH = 199.1 #cm

def load_csv(path:str = None):
    if path is None:
        raise ValueError("Invalid Path")
    try:
        csv = pd.read_csv(path, sep=";")
        return csv
    except Exception as e:
        print(f"Loading CSV failed, check path: {e}")
        return None

def process_and_plot_trajectory(df):
    if df is None:
        return
        
    # Convert all -1 invalid markers to numpy NaN
    df = df.replace(-1, np.nan)

    # Set time as the index (assuming your CSV has a 'time' column)
    if 'time' in df.columns:
        df.set_index('time', inplace=True)

    # Process X
    df['x_front'] = df[['f_ir1', 'f_ir2']].mean(axis=1, skipna=True)
    df['x_back'] = df[['b_ir1', 'b_ir2']].mean(axis=1, skipna=True)
    
    # Invert back sensors and combine
    df['x_back_inv'] = TABLE_LENGTH - df['x_back'] 
    df['x_combined'] = df[['x_front', 'x_back_inv']].mean(axis=1, skipna=True)
    
    # Process Y
    df['y_clean'] = df['y_ult']

    # Interpolate using time index
    df['x_interp'] = df['x_combined'].interpolate(method='index')
    df['y_interp'] = df['y_clean'].interpolate(method='index')

    # Drop trailing/leading NaNs for a clean plot
    df_plot = df[['x_interp', 'y_interp']].dropna()

    # Plot the resulting trajectory
    plt.figure(figsize=(10, 6))
    plt.plot(df_plot['x_interp'], df_plot['y_interp'], marker='o', linestyle='-', color='blue')
    plt.title('Robot Trajectory (Interpolated)')
    plt.xlabel('X Position (cm)')
    plt.ylabel('Y Position (cm)')
    plt.grid(True)
    plt.show()

# --- To use with your actual data ---
# real_df = load_csv('path_to_your_robot_data.csv')
# process_and_plot_trajectory(real_df)