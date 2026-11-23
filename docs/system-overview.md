# System overview

The motor accelerates the flywheel and produces an opposing torque on the base. The change in wheel angular momentum allows the controller to influence base motion. The MATLAB model tracks base angle and rate, wheel motion, and motor current. It also models friction, voltage and current limits, and battery response.

| Subsystem | Design evidence |
| --- | --- |
| Flywheel and motor | The wheel and motor share a vertical axis in the [assembly rendering](../cad/previews/reaction-wheel-full-assembly.png) and [prototype front view](../evidence/photos/prototype-front-view.jpg). |
| Structural stack | The design uses upper and lower decks with supports between them. See the [CAD detail](../cad/previews/deck-stack.png) and [side photo](../evidence/photos/electronics-side-view.jpg). |
| Control electronics | The lab design documentation specifies an Arduino-based setup with a motor driver. The [bench photos](../evidence/README.md) show the wiring and equipment installed below the wheel. |
| Control study | The [MATLAB model](../simulation/README.md) includes open-loop voltage, wheel speed PI, and base-angle PD. |

The project documentation describes a larger laboratory sequence that also includes an inverted-pendulum exercise. This repository covers the reaction-wheel rig and the accompanying model studies.
