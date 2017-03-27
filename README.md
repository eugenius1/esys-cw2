# esys-cw2: Brushless DC Motor Control
Embedded Systems coursework 2 by Team GreenRhino (Patrick Foster, Zoe Williamson and Eusebius Ngemera)

System accepts the following regular expression as a command through the serial input:

```regex
(R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,3})?)?
```

For example, `R-10.5V30` will result in 10.5 backward rotations at an angular speed of 30 rotations per second. We expanded the above requirement to recognise a standalone negative angular speed and to be case insensitive:

```regex
([Rr]-?\d{1,3}(\.\d{1,2})?)?([Vv]-?\d{1,3}(\.\d{1,3})?)?
```

When a speed alone is given, motor rotates forever. When a number of rotations alone is given, motor rotates at maximum speed.

Notes:

- No self tuning of PID parameters (varying inertia)
- No melody playing
- With a fixed number of rotations, wait for motor to stop rotating to complete before giving a new command.

More details are in the [Report](Report.pdf)