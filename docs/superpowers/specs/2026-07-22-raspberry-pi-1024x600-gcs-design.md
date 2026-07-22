# 1024×600 Raspberry Pi GCS Adaptation Design

## Goal

Extend the existing Raspberry Pi display support with a dedicated 1024×600 landscape layout. The new layout keeps all three primary areas visible while preserving the existing 640×480 map-first interface and the full desktop layout.

## Responsive Boundaries

- Widths of 700 CSS pixels or less continue to use the existing compact drawer layout.
- Widths from 701 through 1100 CSS pixels use the new compact three-column layout.
- Widths above 1100 CSS pixels retain the existing desktop layout.
- The page must not overflow horizontally at 1024×600.

## 1024×600 Layout

The intermediate layout keeps the existing left, center, and right panels visible:

1. The left and right panels shrink from 280 pixels to approximately 220 pixels each.
2. The center map consumes the remaining width and remains at least approximately 580 pixels wide at 1024 pixels.
3. Panel padding, section spacing, headings, inputs, and button gaps become slightly denser without changing labels or command hierarchy.
4. The left and right panels scroll vertically and independently when their content exceeds the 600-pixel display height.
5. The document and center map remain fixed to the viewport.
6. Compact drawer close buttons, backdrop, and bottom quick-action bar remain hidden.

## Behavior and Safety

- MAVLink, WebSocket, flight control, waypoint, and mission behavior remain unchanged.
- Existing controls and event handlers are reused without duplication.
- The 640×480 compact drawer behavior remains unchanged.
- The full desktop layout remains unchanged above 1100 pixels.
- Critical controls, including Land, remain available in the left panel and can be reached by panel scrolling when necessary.

## Accessibility and Touch Use

- Controls retain readable labels and visible focus behavior.
- Inputs and buttons remain large enough for practical touchscreen use within the tighter three-column layout.
- Independent panel scrolling must not cause horizontal movement or hide the central map.

## Validation

Automated checks will verify:

- The 701–1100 pixel intermediate media query exists.
- Side panels use the intended compact width and independent vertical scrolling.
- The bottom compact controls remain exclusive to widths of 700 pixels or less.
- Existing 640×480 and desktop structure contracts remain intact.
- Python and JavaScript syntax remain valid.
- All ground-station regression tests pass.

Browser validation will inspect 640×480, 1024×600, and 1280×720 for:

- No horizontal document overflow.
- A nonzero, usable center map area.
- Correct visibility of side panels, drawers, and bottom controls for each breakpoint.
- Independent panel scrolling at 1024×600.
- No browser runtime errors.

## Delivery

The implementation will update `gcs.html`, extend `tests/test_gcs_bridge.py`, create focused commits, and push the result to the repository's configured GitHub remote. Unrelated working-tree changes will remain unstaged.
