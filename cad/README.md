# Mechanical design

The SolidWorks study models a spoked flywheel, vertical motor, rotating deck stack, and lower support. See the [full assembly view](previews/reaction-wheel-full-assembly.png), the [alternate design view](previews/lab-assembly.png), and the [deck-stack detail](previews/deck-stack.png).

## Editable SolidWorks files

Clone or download the complete repository before opening an assembly so its component folders are available.

Start with [Reaction_Wheel_Lab_Assembly.SLDASM](solidworks/Reaction_Wheel_Lab_Project/Reaction_Wheel_Lab_Assembly.SLDASM).

The [main lab project](solidworks/Reaction_Wheel_Lab_Project/) retains the original numbered directories for the layout, ground frame, shaft, rotating base, wheel, motor, encoder, electronics, fasteners, and vendor geometry. The [wheel part](solidworks/Reaction_Wheel_Lab_Project/04_Reaction_Wheel_Subassembly/Reaction_Wheel.SLDPRT) and its [subassembly](solidworks/Reaction_Wheel_Lab_Project/04_Reaction_Wheel_Subassembly/Reaction_Wheel_Subassembly.SLDASM) are in folder `04_Reaction_Wheel_Subassembly`.

Some SolidWorks components link to source files through 3D Interconnect. The lab project's [Raw_STEP folder](solidworks/Reaction_Wheel_Lab_Project/09_Vendor_STEP_Files/Raw_STEP/) contains the coupler and encoder source geometry.

The five original drawing entries were empty placeholders. The available [layout schematic](solidworks/Reaction_Wheel_Lab_Project/10_Drawings/reaction_wheel_next_steps_schematics.png) is included.

## STEP exports

The [STEP folder](step/) contains the 29 lab exports. Open them in a CAD tool that supports STEP. The native SolidWorks files preserve their original feature history and assembly mates. Common parts include the following.

| Geometry | Role in the design |
| --- | --- |
| [Ground base](step/Ground_Base.step) | Stationary support |
| [Lower hardware deck](step/Lower_Hardware_Deck.step) | Lower mounting surface in the rotating stack |
| [Upper motor support deck](step/Upper_Motor_Support_Deck.step) | Motor support and structural mounting |
| [Upper structural retainer](step/Upper_Structural_Retainer.step) | Upper structural interface |
| [Motor cage bracket](step/Motor_Cage_Bracket.step) | Motor mounting |
| [Controller disk guard](step/Controller_Disk_Guard.step) | Guard geometry for the controller disk |

Compare the [physical build views](../evidence/README.md) for the configuration visible on the bench. The exact assembly revision used for the photographed hardware is unconfirmed.
