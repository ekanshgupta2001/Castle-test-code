## TeamCode

This is a code repository made for Rundle Robotics students to help them learn some of the more
advanced concepts of FTC code. It is a working robot program, not a collection of samples: every
file in this module is used by the robot, and the documentation explains why it is written the way
it is.

The code shows how a competitive FTC program is put together: subsystems that own their hardware,
a command scheduler that decides who gets to use them, path following with Pedro Pathing, vision
with a Limelight, sensor fusion for localisation, a small set of one-button macros, and unit tests
that run on a laptop without a robot. Each of those has a short lesson in the `docs/` folder at the
root of the repository.

**Start with `docs/README.md`.** It has a five-minute first read, the lessons in order, the house
rules, and the build and deploy commands.

The stock FTC sample OpModes still live in the `FtcRobotController` module, with their own readme
explaining how to copy one into this module if you ever need it.
