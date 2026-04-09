- [x] Smooth mode control
- [ ] Reverse knob polarity
- [x] Adjustable detend size based on number of steps - done



VIRTUAL HAPTIC CONTROLS:
- [ ] Implement "virtual clicks": If the user moves the knob without snapping to the next step, it counts as an "Enter" command. Moving it in the opposite direction counts as "Back".

- [ ] Implement haptic mode toggling (Smooth mode, Step mode with variable step counts):

    - Double "virtual click": Performed the same as in (1), but triggered twice in rapid succession.
    - OR
    - Long-hold "nudge": If the user maintains torque without snapping to the next step for an extended period, a mode selection menu activates, allowing the user to select their preferred haptic mode.

To make this work in smooth mode, I must implement a virtual detent that activates at the current position whenever the knob is stationary.