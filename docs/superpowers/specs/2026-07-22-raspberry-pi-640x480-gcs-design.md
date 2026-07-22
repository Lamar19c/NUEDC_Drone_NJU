# 640×480 Raspberry Pi GCS Adaptation Design

## Goal

Adapt the existing browser ground control station for a 6-inch Raspberry Pi touchscreen at 640×480 landscape resolution without changing the existing desktop three-column experience or MAVLink behavior.

## Responsive Boundary

- Desktop layout remains unchanged above 700 CSS pixels.
- Compact layout activates at widths of 700 CSS pixels or less.
- The compact layout must work at 640×480 and remain usable at 800×480.
- The page itself must not scroll or overflow horizontally.

## Compact Layout

The compact interface uses a map-first composition:

1. A compact telemetry strip occupies the top row at 34 pixels high.
2. The NED canvas or Leaflet map fills the remaining central workspace.
3. A 48-pixel quick-action bar is fixed to the bottom of the application layout.
4. The existing left panel becomes a left-side drawer for connection and flight controls.
5. The existing right panel becomes a right-side drawer for waypoints and mission controls.

Only one drawer may be open at a time. The active drawer overlays the map instead of reducing its width. A backdrop prevents map interaction while a drawer is open.

## Quick Actions

The compact bottom bar contains five touch targets:

- Control drawer
- Arm
- Takeoff
- Land
- Waypoint drawer

These controls call the existing button handlers. They must not duplicate flight state or introduce a second command implementation. Primary touch targets are at least 40 pixels high.

## Drawer Interaction

- Control and waypoint buttons open their respective drawers.
- Opening one drawer closes the other.
- A close button, backdrop click, or Escape key closes the active drawer.
- The backdrop blocks touch-through to the map.
- Closing a drawer triggers a map/canvas resize refresh.
- The waypoint table scrolls inside the right drawer; the document remains fixed.
- Drawer controls expose their state through `aria-expanded` and drawers use appropriate accessible labels.

## Telemetry Priorities

At compact width, the visible telemetry strip keeps:

- Flight mode and armed state
- Altitude
- Speed
- Battery percentage
- Visible GPS satellite count
- Current mission target

Heading, battery voltage, HDOP, and full coordinates are hidden from the strip at compact width. All telemetry remains available in the unchanged desktop layout.

## Visual Direction

Retain the existing dark, utilitarian flight-console style. The adaptation emphasizes legibility, fast touch recognition, and map visibility rather than introducing a new visual identity. Dangerous actions retain the existing danger color; takeoff and primary actions retain their current hierarchy.

## Safety and Error Behavior

- MAVLink, WebSocket, waypoint, and mission behavior remain unchanged.
- Responsive controls reuse existing command handlers.
- Drawer state is presentation-only and cannot mark the vehicle connected or disconnected.
- Command failures continue to be shown through the existing mission-status area.
- Layout changes must not cover the Land action or make it dependent on an open drawer.

## Validation

Automated/static checks will verify:

- Compact breakpoint and drawer rules are present.
- Required compact controls and accessibility attributes exist.
- Desktop panel structure remains intact.
- No duplicate command implementation is introduced.
- Python and JavaScript syntax remain valid.
- Existing ground-station regression tests still pass.

Browser validation will inspect 640×480, 800×480, and a desktop viewport for:

- No horizontal overflow or clipped primary controls.
- Map/canvas fills the central workspace.
- Drawers are mutually exclusive and dismiss correctly.
- Touch targets remain usable.
- NED canvas and Leaflet map resize after drawer and viewport changes.

## Delivery

The implementation will update `gcs.html`, add or extend regression tests, create a focused commit, and push the resulting commits to the repository's configured GitHub remote. Unrelated firmware build artifacts and user changes will not be staged.
