#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp_weather
# Location to esp-idf-lib directory, change according to your setup
EXTRA_COMPONENT_DIRS := $(HOME)/Coding/esp8266-rtos-sdk_projects/hello_world/components/esp-idf-lib/components/

include $(IDF_PATH)/make/project.mk

