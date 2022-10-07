# DCF77

DCF77-Clock with Multi-LED display with STM32

---

## Decription

This DCF77 Clock not only displays the time and date, but also analyzes and visualizes the signal received.  
The LED rings are placed as such that each LED is on its respective position on a standard clock face.  
The 7 Segment Displays show the time and date in a more readable format.

### Inner LED Ring

The inner LED ring clocks in the data for the current minute telegram. Each "1"-bit will be displayed by the LED lit up, on a "0", the LED is off respectively.

### Outer LED Ring

The outer LED ring gets updated after the current minute data is complete. On the "0-Second"-Mark, the data from the inner ring gets pushed to the outer ring and analyzed. The resulting data will then be displayed on the 7-Segment-Displays as well as the status LEDs. Also at each full hour, the internal RTC gets updated.

### RTC behaviour

1. Initial startup  
After the initial power-up the current RTC value will be pushed to the displays. This time information can only be trusted, once the module has received a valid telegram and was not powered down for more than 3 days.  
2. First received data  
Once the first valid data was received, it will be pushed into the RTC to update it. After successful write, the RTC_LOCK-LED will be on to indicate trustful data stored in the RTC.  
3. On the hour  
To mitigate possible clock drift, the RTC will be updated on the full hour. This ensures up-to-date data stored in it.

---

## Roadmap Hardware

- [x] finalize schematic
- [x] Set footprints
- [x] decide LED layout
  - all on one board
  - ~~seperate frontpanel pcb~~
- [x] Draw Layout
- [x] Review by theBrutzler
- [x] Order Parts
- [x] check footprints
- [x] Order PCB
- [ ] Assemble

---

## Roadmap Software

- [ ] get USB running (for Debug)
- [ ] Get DCF77 Antenna working and bits decoded
- [ ] get MCP7221 running
- [ ] get shift registers running
- [ ] set up flags for stats and data
- [ ] get LED rings running
- [ ] get RTC running
- [ ] Sync everything to start of minute/second
- [ ] get everything running together
