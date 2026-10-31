#!/bin/sh
# Log Pi 5 input voltage + throttle flags once a second (for diagnosing resets under load)
while true; do
  echo "$(date +%T) $(vcgencmd pmic_read_adc EXT5V_V | grep -o "[0-9.]*V$") $(vcgencmd get_throttled)"
  sleep 1
done
