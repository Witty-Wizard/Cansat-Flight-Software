![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/Gagan-Space/Cansat-Flight-Software/test.yml)
![GitHub Release](https://img.shields.io/github/v/release/Gagan-Space/Cansat-Flight-Software)
![GitHub License](https://img.shields.io/github/license/Gagan-Space/Cansat-Flight-Software)
![GitHub Repo stars](https://img.shields.io/github/stars/Gagan-Space/Cansat-Flight-Software?style=flat)

# Cansat Flight Software
This repository contains the flight software for a CanSat project. The software is designed to run on an ESP32 microcontroller using the Arduino framework and PlatformIO.
## Overview

This flight software is developed for a CanSat. The software handles the operation of the CanSat's sensors, communication with the ground station, and other necessary tasks.

## ESP32 Pinout
![Esp32 Pinout](esp32-pico-kit-1-pinout.png)

## Features
Flight Software has the following features:
- Integration with BNO085 and DPS310 sensors for accurate orientation and altitude measurement.
- Real-time data transmission to the ground station.
- Error handling and fault tolerance mechanisms.

with many more features on the way!

## Getting Started

### Setting up Development Enviorment
To set up your development environment, follow these steps:

1. Install Visual Studio Code (VS Code):
- Download and install VS Code from the [official website](https://code.visualstudio.com/).

2. Install PlatformIO Extension:
- Open VS Code.
- Go to the Extensions view by clicking on the square icon in the sidebar or pressing Ctrl+Shift+X.
- Search for "PlatformIO" in the Extensions Marketplace.
- Click on "Install" to install the PlatformIO extension.
- Restart VS Code after installation is complete.


### Cloning the Repository
To get started with the project, follow these steps:

1. Clone the repository to your local machine:

    ```bash
    git clone https://github.com/Gagan-Space/Cansat-Flight-Software.git
    ```
2. Open the project in Visual Studio Code (VS Code):

- Launch VS Code.
- Click on "File" > "Open Folder...".
- Navigate to the directory where you cloned the repository (Cansat-Flight-Software) and select it.
- The project will open in VS Code, and you can start working on it.

## Contributing

We welcome contributions from the community! If you'd like to contribute, please follow these guidelines:

1. Fork the repository
2. Create your feature branch: 
    ```bash
    git checkout -b feature-name`
    ```
3. Commit your changes:
    ```bash
    git commit -am 'Add some feature'
    ```

4. Push to the branch: 
    ``` bash
    git push origin feature-name
    ```
5. Submit a pull request

For major changes, please open an issue first to discuss what you would like to change.