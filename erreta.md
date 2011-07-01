Mini Trigger Box Erreta
========================================================================

Rear panel
------------------------------------------------------------------------

- The small toggle switch is too flimsy. The solder lugs easily loosen.
- Although the Hirose connectors work well when installed, they are very
  expensive and a bit hard to solder. RJ-45 sockets might be better.
- The Hirose connectors' pin order has to be reversed because of an
  error with the termination of the cables.

Front panel
------------------------------------------------------------------------

- The holes in the LCD PCB needed to be widened because it was hard to
  find screws and offsets that were small enough.
- Some of the pushbuttons used have to be pressed quite hard to
  register.
- The LCD is very exposed, and leaves a gap in the front panel. A cover
  in front of the LCD would prevent this.

Wiring
------------------------------------------------------------------------

- The rainbow colored ribbon cables are awful to work with when
  separating the individual leads. The grey ones are perfect.
- The power supply cables are a lot thicker than they need to be.

Enclosure
------------------------------------------------------------------------

- The holes for screws, connectors etc. are too time-consuming to be
  made by hand, and should be CNC milled.
- Making lables by hand is too time-consuming. Labels should be machine
  printed.

Power supply
------------------------------------------------------------------------

- It is way too big, powerful and expensive for the load it has to
  handle.
- The dual voltage output is convenient, but a lot of money could
  probably be saved by using an on-board linear voltage regulator.

Printed circuit board
------------------------------------------------------------------------

- The current opto-isolation design is flawed. A workaround is bypassing
  the output optocoupler.
- Sockets should be used for the opto-couplers in case they need to be
  replaced.
- Triggering output should be amplified to work over long distances.
  Experiments on this needs to be done, though. One should investigate
  if using fiber optics is a viable alternative. Advantages include the
  best kind of electrical isolation, and good long-distance performance.
- An ISP header should be added, perhaps with an external connector.
- The debugging LED is too close to the microcontroller. This means that
  is has to be bent to be able to remove the microcontroller. It should
  be replaced by an SMD LED anyway.
- The pull-up resistors for the buttons are redundant as the
  microcontroller has built-in ones.
- The microcontroller is considered obsolete, and should be replaced by
  a newer one.
- The footprint used for the crystal capasitors are through-hole, and
  not SMD. They should be SMD.

General
------------------------------------------------------------------------

- The "camera connected" feature has yet to be tested.
- The firmware lacks keylock, and LCD brightness and contrast adjustment
  features.
- There are no external serial ports for remote administration.
