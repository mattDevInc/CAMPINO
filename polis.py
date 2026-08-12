import numpy as np
import matplotlib.pyplot as plt

def generate_wiggle_coordinates(start=0, end=2*np.pi*3, points=1000):
    """
    Calculates the spatial points for the 3D parametric line.
    
    Parameters:
    - start: The beginning x-coordinate value.
    - end: The ending x-coordinate value.
    - points: Total number of data points to generate for a smooth curve.
    """
    # Linear forward motion along the x-axis
    x = np.linspace(start, end, points)
    
    # Left and right oscillation along the y-axis
    y = np.sin(x)
    
    # Up and down oscillation along the z-axis (frequency doubled)
    z = np.cos(2 * x)
    
    return x, y, z

def main():
    # 1. Generate the line data coordinates
    x, y, z = generate_wiggle_coordinates()

    # 2. Configure the 3D visualization window
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # 3. Plot the parametric line with customized styling
    ax.plot(x, y, z, color='#1f77b4', linewidth=2.5, label='Parametric Path')

    # 4. Define titles and coordinate axis labels
    ax.set_title('3D Line Oscillation (Parametric Curve)', fontsize=14, pad=20)
    ax.set_xlabel('X Axis (Forward Position)', fontsize=10)
    ax.set_ylabel('Y Axis (Left / Right Wiggle)', fontsize=10)
    ax.set_zlabel('Z Axis (Up / Down Wiggle)', fontsize=10)

    # 5. Fine-tune the grid lines and initial viewing perspective
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.view_init(elev=20, azim=-60)
    ax.legend(loc='upper left')

    # 6. Render and display the interactive plot window
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
