# DewPointMonitor

DewPointMonitor is a project designed for monitoring dew point and notifying users when the dew point is reached to prevent condensation on telescope lenses and other sensitive equipment.

## Current Project Status

The project is in development stage. The core functionalities are implemented, and drivers for the DHT11 sensor are not ready.

## Features

- Measures temperature and humidity using DHT11 sensor.
- Calculates dew point.
- Notifies the user with a buzzer when the dew point is reached.
- Displays information on LCD display via I2C interface.

## Components

- STM32F411 Nucleo board
- LCD display
- DHT11 temperature and humidity sensor
- I2C module for LCD
- Active buzzer

## Components Connection

- **DHT11 Data Output** -> PB13
- **Active Buzzer** -> PB14
- **Display I2C Clock** -> PB6
- **Display I2C SDA** -> PB7

## Installation and Configuration

1. **Clone the repository:**
    ```sh
    git clone https://github.com/yourusername/DewPointMonitor.git
    ```

2. **Open the project in STM32CubeIDE.**

3. **Compile and flash the firmware to the STM32F411 Nucleo board.**

4. **Connect the components according to the schematic:**
    - Connect the DHT11 sensor to the corresponding GPIO pins.
    - Connect the I2C module for the LCD display to the I2C pins.
    - Connect the buzzer to a GPIO pin.

## Usage

1. Power on the STM32F411 Nucleo board.
2. The current temperature and humidity will be displayed on the LCD.
3. The system will calculate the dew point in real-time.
4. When the temperature reaches the dew point, the buzzer will sound.

## Applications

- Preventing condensation on telescope lenses.
- Monitoring indoor environments to prevent mold formation.
- Industrial applications where dew point monitoring is critical.

## Contribution

Contributions are welcome! Please fork this repository and submit a pull request.

## License

This project is licensed under the terms of the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

If you have any questions or suggestions, please contact me at max.podloobniy@gmail.com.
