# TO DO

- [x] Smooth mode control
- [ ] Reverse knob polarity
- [x] Adjustable detend size based on number of steps - done



## VIRTUAL HAPTIC CONTROLS

- [ ] Implement "virtual clicks": If the user moves the knob without snapping to the next step, it counts as an "Enter" command. Moving it in the opposite direction counts as "Back".
- [ ] Remove confirmation via physical button (btn2) and keep confirm/back fully on haptic virtual controls.

- [ ] Implement haptic mode toggling (Smooth mode, Step mode with variable step counts):
    - Double "virtual click": Performed the same as in (1), but triggered twice in rapid succession.
    - OR
    - Long-hold "nudge": If the user maintains torque without snapping to the next step for an extended period, a mode selection menu activates, allowing the user to select their preferred haptic mode.

To make this work in smooth mode, I must implement a virtual detent that activates at the current position whenever the knob is stationary.


## Custom Board

- [ ] make custom board devicetree
- [ ] swap AS5048 with TMAG5170 - should not be difficuil - Zephyr has TMAG5170 driver
- [ ] TMC6300 -> DRV8311H
    - 6pwm -> 3pwm
    - INL control
    - SOx analog signals
    - nSLEEP, nFAULT integration
- [ ] MAX17260
    - existing driver MAX17262 - has internal sense rezistor - extarnal has to be defined in devicetree
    - ALRT
