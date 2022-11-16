# ESP Weather

Simple weather station based on ESP8266 chip. It uses BME280 and BH1750 sensors to collect environment data and transmit them over MQTT to the broker using TLS (mutual auth). It also utilizes MQTT authentication when connecting to the broker. This adds another layer of security and allows to easily identify clients.

This project uses the [esp-idf-lib](https://github.com/UncleRus/esp-idf-lib) to interface with BME280 and BH1750 sensors.

## Hardware configuration

To configure I2C lines, run `make menuconfig` and define I2C bus lines under `BME280 configuration`. By default **I2C_SCL is pin 4** and **I2C_SCA is pin 5.** Also depending on what addresses your sensors register to the I2C bus, you can change them accordingly in menuconfig.

## Minimal configuration

In order to run the project, you need to run `make menuconfig` and configure the following:
- WIFI SSID and password
- MQTT broker URI (defaults to `mqtt://mqtt.eclipse.org:1883`)
- MQTT client username and password
- MQTT topic where client publishes the data

## WIFI configuration

Run `make menuconfig` to enter your WIFI SSID and password credentials.

## MQTT configuration

Run `make menuconfig` to define MQTT broker URI (defaults to `mqtt://mqtt.eclipse.org:1883`), client username and password that are required to authenticate to the broker when connecting to it.

## Payload format

ESP produces a JSON payload containing temperature, humidity, pressure and light values in such format:

```
{
    "temperature": "2594",
    "humidity": "67174",
    "pressure": "252160",
    "light": "523"
}
```

NOTE: All payload values are send in a string format in order to save processing power on an ESP8266 (by default float procesing is disabled on them). Therefore, it is needed to parse those values back to ints and then convert them as follows:

```
temperature / 100 (Celsius)
humidity / 1024 (%H)
pressure / 256 (Pascal)
light / 1 (lux)
```

## TLS connection

In order to connect ESP MQTT client, an MQTT broker is needed as well as a CA. Use [this guide](http://www.steves-internet-guide.com/mosquitto-tls/) to create CA and generate appropriate server and client certificates to establish secure connection. 

After creating certificates, place CA certificate, client certificate and client key (PEM format) in the project main directory.

## TODOs

- sometimes the MQTT fails to exchange data even though the certificates were verified and the connection was established (esp-tls-mbedtls error: -0x4c, very rare)
- when any error occurs, immediately retry to publish MQTT data again 
