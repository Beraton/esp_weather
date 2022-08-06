#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp_weather
# Location to esp-idf-lib directory, change according to your setup
EXTRA_COMPONENT_DIRS := $(HOME)/Coding/esp8266-rtos-sdk_projects/esp_weather/components/esp-idf-lib/components
EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip max31865 ls7366r max31855
						

include $(IDF_PATH)/make/project.mk

