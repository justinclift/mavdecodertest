A very simple MAVLink decoder, based on gomavlib, to display the
details of MAVLink messages passing through it.

eg:

```
System status message from drone
Mainloop time used 35%, batt voltage: 0, batt current: -1, batt remain: -1
Comm drop rate: 0, comm errors: 0

Power status message from drone. 5V rail voltage: 4.90v, Servo rail voltage: 4.86v, flags: 0

Memory info message from drone. Heap top: 0, free mem: 46896, free mem32: 0

Drone message status current mission is # 0
```

It could do with nicer display and some clean up, but this fulfills
it's purpose as-is for now.
