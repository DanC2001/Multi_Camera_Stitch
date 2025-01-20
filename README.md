# Project Title

-- Full transparancy - Alot of these instructions are chatGPT as i don't have the time to write out fully. 

This repository is built around a development container and Visual Studio Code (VSCode) to provide a consistent and isolated development environment for your ROS2 Humble project.

## Requirements

To use these instructions, ensure you have the following installed on your machine:

- **Docker**: [Download and install Docker](https://www.docker.com/get-started)
- **Visual Studio Code (VSCode)**: [Download and install VSCode](https://code.visualstudio.com/)
  - **Dev Containers Extension**: Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension by Microsoft.

## Building the Docker Image

First, build the Docker image using the provided Dockerfile in the `.devcontainer` directory.

```bash
cd <path_to_project>/
docker build .devcontainer/. -t dc_abyss:latest
```

## Opening the Development Container in VSCode

Once you have successfully built the Docker image, follow these steps to open the project within the development container using Visual Studio Code:

### 1. Open Visual Studio Code

Launch VSCode on your machine.

### 2. Install the Dev Containers Extension

Ensure that the **Dev Containers** extension is installed:

- Go to the Extensions view by clicking on the **Extensions** icon in the Activity Bar on the side of VSCode or by pressing `Ctrl+Shift+X` (`Cmd+Shift+X` on macOS).
- Search for "**Dev Containers**" and install the extension provided by Microsoft.

### 3. Open the Project Folder

- Click on `File` > `Open Folder...` (or `Open...` on macOS).
- Navigate to and select the root directory of your project.

### 4. Reopen in Container

- Click on the green `><` icon located at the bottom-left corner of the VSCode window.
- From the dropdown menu, select `Reopen in Container`.

  *Alternatively, you can use the Command Palette:*

  - Press `F1` (or `Ctrl+Shift+P` / `Cmd+Shift+P` on macOS) to open the Command Palette.
  - Type `Remote-Containers: Reopen in Container` and select it from the list.

### 5. Select the Existing Docker Image

If prompted to select a container configuration:

- Choose to **Use an Existing Docker Image**.
- Select `dc_abyss:latest` from the list of available images.

### 6. Wait for the Container to Initialize

VSCode will now set up the development environment inside the Docker container. This process may take a few minutes, especially during the first setup. You'll see progress notifications in the bottom status bar.

### 7. Verify the Development Environment

Once the container is up and running:

- The bottom-left corner of VSCode should indicate that you're connected to the container.
- You can open a terminal within VSCode (`Terminal` > `New Terminal`) to verify that you're inside the Docker container environment.
- All extensions and settings defined in the `.devcontainer` configuration will be applied.

### 8. Start Developing

You are now ready to develop within the isolated and consistent development environment provided by the Docker container. You can run, debug, and test your ROS2 applications seamlessly.

---

### Additional Tips

#### Managing the Container

- **Rebuild the Container:**
  
  To rebuild the container (e.g., after making changes to the Dockerfile or `devcontainer.json`), click on the green `><` icon and select `Rebuild Container`.

- **Close the Container:**
  
  To close the container and return to your local environment, select `Close Remote Connection` from the same menu.

#### Persisting Data

Ensure that any necessary data or configurations are properly mounted or persisted using Docker volumes as defined in your `.devcontainer` setup.

#### Customizing the Dev Container

You can customize the development environment by modifying the `.devcontainer/devcontainer.json` file to include additional extensions, settings, or Docker configurations as needed.

---

By following these steps, you should be able to seamlessly open and work within the development container using VSCode, leveraging the pre-configured Docker environment for your ROS2 Humble project.

## Running the Node

This section guides you through setting up and running the necessary nodes for your ROS2 project, including the Foxglove Bridge, replaying a bag file, and starting the multi-camera module.

### 1. Launch the Foxglove Bridge

The Foxglove Bridge facilitates easier introspection by creating a WebSocket connection that Foxglove Studio can use.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 2. Connect Foxglove Studio to the Bridge

If you have Foxglove Studio installed, follow these steps to connect it to the Foxglove Bridge:

1. **Open Foxglove Studio**

   Launch Foxglove Studio on your machine.

2. **Add a New Connection**

   - Click on the **Open Connection** button.
   - Select **Foxglove WebSocket** from the list of available connection types.

3. **Configure Connection Settings**

   - **URL:** If you haven't changed the settings before, the default URL should work (e.g., `ws://localhost:8765`).
   - **Name:** Optionally, give your connection a meaningful name for easy reference.

4. **Connect**

   Click the **Connect** button to establish the connection. Once connected, you should see data streams appearing in Foxglove Studio.


### 3. Replay the Bag File

Replaying a ROS2 bag file allows you to simulate sensor data and other ROS messages for testing and development purposes.

```bash
ros2 bag play ./bag_files/case_study/case_study.db3 --loop
```

- **`--loop`**: This flag enables continuous replay of the bag file, allowing you to repeatedly test your setup without manually restarting the replay.

### 4. Start the Multi-Camera Module

The `multi_camera_node` remaps topics from the bag file to simplified `camera_X/image` topics, facilitating easier data handling and processing.

```bash
ros2 run dc_abyss_solutions_test multi_camera_node --ros-args \
    --remap /camera_1/image:=/platypus/camera_1/dec/manual_white_balance \
    --remap /camera_2/image:=/platypus/camera_2/dec/manual_white_balance \
    --remap /camera_3/image:=/platypus/camera_3/dec/manual_white_balance
```

#### Breakdown of the Command:

- **`ros2 run dc_abyss_solutions_test multi_camera_node`**: Executes the `multi_camera_node` from the `dc_abyss_solutions_test` package.
- **`--ros-args`**: Passes additional ROS arguments to the node.
- **`--remap`**: Remaps the original topic names to simplified names for easier access and management.

  - **Example Remaps:**
    - `/camera_1/image` → `/platypus/camera_1/dec/manual_white_balance`
    - `/camera_2/image` → `/platypus/camera_2/dec/manual_white_balance`
    - `/camera_3/image` → `/platypus/camera_3/dec/manual_white_balance`

### Summary of Steps

1. **Launch the Foxglove Bridge** to establish a WebSocket connection.
2. **Connect Foxglove Studio** to the Foxglove Bridge for data visualization.
3. **Replay the ROS2 bag file** to simulate sensor data and ROS messages.
4. **Start the Multi-Camera Module** to remap and simplify topic names for easier data handling.

---

### Additional Tips

- **Ensure Docker and VSCode Dev Containers are Running Properly:**
  
  Before running these nodes, make sure your development container is up and running to provide the necessary environment and dependencies.

- **Monitor ROS2 Topics:**
  
  Use tools like `ros2 topic list` and `ros2 topic echo` to monitor active topics and ensure that remappings are functioning as expected.

- **Troubleshooting Connections:**
  
  If Foxglove Studio fails to connect, verify that the Foxglove Bridge is running and that the WebSocket URL and port settings are correct.

---

By following these steps, you will have a fully functional setup that leverages the Foxglove Bridge for data visualization, replays ROS2 bag files for testing, and simplifies topic management with the multi-camera module.
