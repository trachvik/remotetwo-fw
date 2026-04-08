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

### Tool select

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
