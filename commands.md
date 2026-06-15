# Commands

## Protocol Frame

All outbound commands from nRF5340 are automatically framed by firmware:

```text
cmd:<id>:<payload>
```

Examples:

```text
cmd:17:mv:x:-10.0
cmd:18:tp:e:210
cmd:19:l:on
```

`id` is generated on nRF5340 and used to match ACK from Raspberry Pi.

## Output Commands (nRF5340 -> Raspberry Pi)

### Position (relative)

```text
mv:x:-10.0  //relative move of -10 units on X
mv:y:15.0   //relative move of +15 units on Y
mv:z:5.0    //relative move of +5 units on Z
mv:e:20.0   //relative move of +20 units on E
```

### Temperature (absolute target)

```text
tp:e:210    //set extruder target temperature
tp:b:60     //set bed target temperature
```

### Tool select (MMU)

```text
t:0
t:1
..
t:8
```

### Printer light

```text
l:on
l:off
```

### Z-offset (relative)

```text
offset:+0.10
offset:-0.05
```

### Print sheet macro select

```text
s:custom0
s:custom1
s:custom2
```

### Home

```text
home:all    //G28
home:x      //G28 X
home:y      //G28 Y
home:z      //G28 Z
```

### Motors off

```text
motors:off  //M84
```

### Fan speed

```text
fan:<percent>   //0-100, mapped to M106 S0-255
```

### Cool down

```text
cool:down   //TURN_OFF_HEATERS
```

### Filament

```text
filament:preheat:pla
filament:preheat:petg
filament:load
filament:unload
```

### Calibration

```text
calib:z
calib:bed_mesh
calib:first_layer
calib:probe
```

### MMU

```text
mmu:home
mmu:resume
mmu:locate:<0-8>   //locate selector to tool slot
```

### Printing

```text
print:pause
flow:<percent>    //M221 S<percent>
speed:<percent>   //M220 S<percent>
```

### Fake position

```text
fake:position   //SET_KINEMATIC_POSITION X=150 Y=150 Z=10
```

### Physical hot-key buttons

Four physical buttons send a portable button-ID command. The remote only reports
which button was pressed; the concrete G-code is assigned on the gateway side
(overridable via `--custom1-gcode` … `--custom4-gcode` CLI flags).

```text
btn:custom1   // btn_north-west  – default: PREHEAT_PLA
btn:custom2   // btn_north-east  – default: G28 (home all axes)
btn:custom3   // btn_south-east  – default: TOGGLE_LIGHT (light on/off)
btn:custom4   // btn_west        – default: M84 (motors off)
```

Button-to-GPIO mapping (from `remotetwo_nrf5340_cpuapp_ns.overlay`):

| Command      | Label            | GPIO   | INPUT_KEY |
|-------------|-----------------|--------|-----------|
| btn:custom1 | btn_north-west  | P0.20  | KEY_4     |
| btn:custom2 | btn_north-east  | P1.12  | KEY_6     |
| btn:custom3 | btn_south-east  | P0.19  | KEY_7     |
| btn:custom4 | btn_west        | P1.08  | KEY_3     |

## Input Commands (Raspberry Pi -> nRF5340)

`state:` is the preferred prefix. For compatibility during testing, `status:` is accepted as an alias.

### ACK / deny

```text
ack:<id>:ok
ack:<id>:deny:<reason>
```

Examples:

```text
ack:17:ok
ack:18:deny:not_homed
ack:19:deny:soft_limit
```

### State telemetry

```text
state:can_move:<0|1>
state:printing:<0|1>
state:tool:<n>
state:homed:x:<0|1>:y:<0|1>:z:<0|1>
state:pos:x:<float>:y:<float>:z:<float>:e:<float>
state:temp:e:<float>:b:<float>
```

Legacy-compatible aliases accepted by firmware:

```text
status:can_move:<0|1>
status:printing:<0|1>
status:tool:<n>
status:homed:x:<0|1>:y:<0|1>:z:<0|1>
status:pos:x:<float>:y:<float>:z:<float>:e:<float>
status:temp:e:<float>:b:<float>
```

Examples:

```text
state:can_move:1
state:homed:x:1:y:1:z:1
state:pos:x:123.40:y:55.20:z:0.30:e:12.10
state:temp:e:201.5:b:59.0
```

## Notes

- Movement commands (`mv:*` and `offset:*`) are blocked on nRF5340 when last known `state:can_move` is `0`.
- If no `state:*` telemetry has been received yet (e.g., mobile-only testing), movement is allowed by default.
- After Raspberry Pi telemetry is implemented and stable, keep movement blocking enabled and driven by `state:can_move` for safe operation.
- In axis edit mode, the shown value starts from last known absolute position, but transmitted `mv:*` is still relative delta.
- In position edit mode (`X/Y/Z/E`), each knob step sends `mv:*` immediately; confirm button only exits edit mode.
- Definitions for custom keys and sheets remain managed on Raspberry Pi (Flask side).
