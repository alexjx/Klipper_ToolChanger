# Four physical tools

This example shows four independently configured physical extruder tools. The
tools share pickup/dropoff actions and thermal timer defaults through
`[toolgroup 0]`, while each `[tool N]` owns its resources, dock coordinates,
offset, retraction, pressure advance and temperature defaults.

Add both files to `printer.cfg`:

```
[include custom/tools.cfg]
[include custom/tool_macro.cfg]
```

The motion macros assume a Jubilee-style row of docks on the right side of the
machine. Treat the coordinates, speeds and lock mechanism as examples; verify
them for your hardware before moving the printer.

All four tools are physical. Virtual tools, physical-parent inheritance and
runtime tool remapping are intentionally not part of this framework.
