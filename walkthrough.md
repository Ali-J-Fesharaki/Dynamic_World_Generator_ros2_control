# UI Architecture Redesign

The application has been completely redesigned to replace the linear, step-by-step `QWizard` with a professional, non-linear desktop application shell. This allows you to freely switch between designing walls, placing obstacles, and configuring motion paths without being forced through a strict sequence.

## Key Changes

### 1. Modern Application Shell
- **Main Window (`MainWindow`)**: Replaced the wizard with a main window that features a vertical icon-based sidebar for navigation.
- **Top Bar & Status Bar**: 
  - The top bar displays the current loaded world and simulation engine (Gazebo Harmonic/Fortress).
  - The bottom status bar shows your cursor coordinates on the canvas and feedback messages for your actions (e.g., "Wall created", "Export successful").
- **Splash Screen**: A beautiful dark-themed overlay with an animated GIF and version badge greets the user on startup, fading out when clicked.

### 2. Unified Obstacles Panel
- Merged the "Static Obstacles" and "Dynamic Obstacles" pages into a single **Obstacles Panel**.
- You can now add an obstacle (Box, Cylinder, Sphere), assign its dimensions, pick a color, and place it on the canvas.
- Once selected from the list, you can immediately define a **Motion Path** (Linear, Elliptical, Polygon) for that exact obstacle within the same view.

### 3. Setup and Export Panels
- **Setup Panel**: Merges the welcome and simulation selection steps. You can pick your Gazebo version and instantly create or load a world. Loading a world unlocks the rest of the application.
- **Export Panel**: Replaces the old "Coming Soon" page. It provides a summary of the loaded world (count of walls, static obstacles, and dynamic obstacles), quick action buttons to export YAML or launch Gazebo, and frosted-glass "Coming Soon" cards for future roadmap features (like Gazebo Ionic and Isaac Sim).

### 4. Shared Application State
- Created an `AppState` object that holds the `WorldManager` and the canvas `QGraphicsScene`. This state is passed to all panels, eliminating the messy `self.wizard()` references and making the codebase much more modular.

## Next Steps
You can run the application normally using:
```bash
python3 code/dwg_wizard.py
```

Try clicking through the new sidebar, creating a world in the Setup panel, and then using the unified Obstacles panel to place shapes and draw motion paths!
