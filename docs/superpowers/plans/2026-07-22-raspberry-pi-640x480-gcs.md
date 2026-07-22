# Raspberry Pi 640×480 GCS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a map-first, touch-friendly 640×480 layout to the existing GCS while preserving its desktop layout and flight-control behavior.

**Architecture:** Keep `gcs.html` as the single frontend file. Add compact-only controls that forward clicks to existing controls, CSS media queries that convert the two side panels into overlay drawers, and a small drawer state controller that keeps presentation state separate from MAVLink state.

**Tech Stack:** HTML5, CSS media queries, vanilla JavaScript, Python `unittest`, Node.js syntax validation

---

## File Structure

- Modify `gcs.html`: responsive styles, drawer markup, compact quick-action bar, and drawer controller.
- Modify `tests/test_gcs_bridge.py`: static regression checks for breakpoint, compact controls, accessibility state, handler reuse, and desktop structure preservation.
- Use `docs/superpowers/specs/2026-07-22-raspberry-pi-640x480-gcs-design.md` as the accepted design contract.

### Task 1: Lock the compact layout contract with failing tests

**Files:**
- Modify: `tests/test_gcs_bridge.py`
- Test: `tests/test_gcs_bridge.py`

- [ ] **Step 1: Add a reusable HTML source helper**

Add this method to `GroundStationRegressionTests`:

```python
def html_source(self):
    return (ROOT / "gcs.html").read_text(encoding="utf-8")
```

- [ ] **Step 2: Add compact-layout structure tests**

```python
def test_compact_layout_contract_is_present(self):
    html = self.html_source()
    self.assertIn("@media (max-width: 700px)", html)
    self.assertIn('id="compact-controls"', html)
    self.assertIn('id="drawer-backdrop"', html)
    self.assertIn('id="compact-open-controls"', html)
    self.assertIn('id="compact-open-waypoints"', html)
    self.assertIn('aria-expanded="false"', html)

def test_desktop_panel_structure_is_preserved(self):
    html = self.html_source()
    self.assertEqual(html.count('id="left-panel"'), 1)
    self.assertEqual(html.count('id="center-panel"'), 1)
    self.assertEqual(html.count('id="right-panel"'), 1)
```

- [ ] **Step 3: Add command-forwarding safety tests**

```python
def test_compact_flight_controls_forward_to_existing_buttons(self):
    html = self.html_source()
    self.assertIn('forwardCompactAction("btn-arm")', html)
    self.assertIn('forwardCompactAction("btn-takeoff")', html)
    self.assertIn('forwardCompactAction("btn-land")', html)
    self.assertNotIn('wsSend("arm")', compact_script_segment)
    self.assertNotIn('wsSend("takeoff")', compact_script_segment)
    self.assertNotIn('wsSend("land")', compact_script_segment)
```

Extract `compact_script_segment` between `// ============== Compact layout ==============` and `// ============== Init ==============`.

- [ ] **Step 4: Run the new tests and verify RED**

Run:

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
python -m unittest tests.test_gcs_bridge.GroundStationRegressionTests.test_compact_layout_contract_is_present tests.test_gcs_bridge.GroundStationRegressionTests.test_desktop_panel_structure_is_preserved tests.test_gcs_bridge.GroundStationRegressionTests.test_compact_flight_controls_forward_to_existing_buttons -v
```

Expected: compact-layout tests fail because the new controls and media query do not exist; desktop structure test passes.

### Task 2: Add compact markup and responsive layout

**Files:**
- Modify: `gcs.html`
- Test: `tests/test_gcs_bridge.py`

- [ ] **Step 1: Add drawer accessibility attributes to existing panels**

Change the panel openings to:

```html
<div id="left-panel" class="panel compact-drawer" role="dialog" aria-label="连接与飞行控制">
<div id="right-panel" class="panel compact-drawer" role="dialog" aria-label="航点与任务控制">
```

Add a `.compact-drawer-close` button as the first child of each panel. It remains hidden on desktop.

- [ ] **Step 2: Add compact backdrop and quick controls**

Insert after `#app` content and before the scripts:

```html
<button id="drawer-backdrop" type="button" aria-label="关闭侧边面板"></button>
<nav id="compact-controls" aria-label="飞行快捷控制">
  <button id="compact-open-controls" type="button" aria-controls="left-panel" aria-expanded="false">控制</button>
  <button id="compact-arm" type="button">解锁</button>
  <button id="compact-takeoff" type="button">起飞</button>
  <button id="compact-land" type="button">降落</button>
  <button id="compact-open-waypoints" type="button" aria-controls="right-panel" aria-expanded="false">航点</button>
</nav>
```

- [ ] **Step 3: Add compact CSS**

Add base rules that hide compact-only controls on desktop, then add `@media (max-width: 700px)` rules that:

```css
@media (max-width: 700px) {
  #app { position: relative; height: 100dvh; padding-bottom: 48px; }
  #center-panel { width: 100%; }
  #telemetry-bar { order: -1; height: 34px; flex-wrap: nowrap; overflow: hidden; padding: 3px 5px; }
  #t-heading, #t-bat, #t-hdop, #t-pos { display: none; }
  #left-panel, #right-panel {
    position: fixed; top: 0; bottom: 48px; z-index: 1200;
    width: min(78vw, 320px); min-width: 0; transform: translateX(-105%);
    transition: transform 160ms ease; box-shadow: 8px 0 24px rgba(0,0,0,.45);
  }
  #right-panel { right: 0; left: auto; transform: translateX(105%); }
  #left-panel.drawer-open, #right-panel.drawer-open { transform: translateX(0); }
  #drawer-backdrop.drawer-open { display: block; }
  #compact-controls { display: grid; grid-template-columns: repeat(5, 1fr); height: 48px; }
  #compact-controls button { min-height: 44px; }
}
```

Hide telemetry by adding stable wrapper classes or `data-compact-hide` attributes to the corresponding `.telem-item` containers rather than hiding value spans alone.

- [ ] **Step 4: Run tests and verify GREEN for structure**

Run the three Task 1 tests. Expected: all pass.

### Task 3: Implement drawer state and command forwarding

**Files:**
- Modify: `gcs.html`
- Test: `tests/test_gcs_bridge.py`

- [ ] **Step 1: Add a presentation-only drawer controller**

Add before the Init section:

```javascript
// ============== Compact layout ==============
var compactDrawer = null;

function compactLayoutActive() {
  return window.matchMedia("(max-width: 700px)").matches;
}

function refreshResponsiveMap() {
  window.setTimeout(function() {
    redrawNEDCanvas();
    if (leafletMap) leafletMap.invalidateSize();
  }, 180);
}

function closeCompactDrawers() {
  ["left-panel", "right-panel"].forEach(function(id) {
    document.getElementById(id).classList.remove("drawer-open");
  });
  document.getElementById("drawer-backdrop").classList.remove("drawer-open");
  document.getElementById("compact-open-controls").setAttribute("aria-expanded", "false");
  document.getElementById("compact-open-waypoints").setAttribute("aria-expanded", "false");
  compactDrawer = null;
  refreshResponsiveMap();
}

function openCompactDrawer(panelId, triggerId) {
  if (!compactLayoutActive()) return;
  closeCompactDrawers();
  document.getElementById(panelId).classList.add("drawer-open");
  document.getElementById("drawer-backdrop").classList.add("drawer-open");
  document.getElementById(triggerId).setAttribute("aria-expanded", "true");
  compactDrawer = panelId;
}

function forwardCompactAction(targetId) {
  document.getElementById(targetId).click();
}
```

- [ ] **Step 2: Wire drawer and compact buttons**

```javascript
document.getElementById("compact-open-controls").onclick = function() {
  openCompactDrawer("left-panel", "compact-open-controls");
};
document.getElementById("compact-open-waypoints").onclick = function() {
  openCompactDrawer("right-panel", "compact-open-waypoints");
};
document.getElementById("compact-arm").onclick = function() { forwardCompactAction("btn-arm"); };
document.getElementById("compact-takeoff").onclick = function() { forwardCompactAction("btn-takeoff"); };
document.getElementById("compact-land").onclick = function() { forwardCompactAction("btn-land"); };
document.getElementById("drawer-backdrop").onclick = closeCompactDrawers;
document.querySelectorAll(".compact-drawer-close").forEach(function(button) {
  button.onclick = closeCompactDrawers;
});
window.addEventListener("keydown", function(event) {
  if (event.key === "Escape") closeCompactDrawers();
});
window.matchMedia("(max-width: 700px)").addEventListener("change", closeCompactDrawers);
```

- [ ] **Step 3: Run all regression tests**

Run:

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
python -m unittest discover -s tests -v
```

Expected: all tests pass.

### Task 4: Validate syntax and 640×480 behavior

**Files:**
- Modify if required: `gcs.html`

- [ ] **Step 1: Validate Python and JavaScript syntax**

Run:

```powershell
python -c "import ast, pathlib; ast.parse(pathlib.Path('gcs_bridge.py').read_text(encoding='utf-8')); print('Python syntax OK')"
node -e "const fs=require('fs'); const h=fs.readFileSync('gcs.html','utf8'); const s=h.match(/<script>([\s\S]*)<\/script>/)[1]; new Function(s); console.log('JavaScript syntax OK');"
```

Expected: both syntax checks print `OK`.

- [ ] **Step 2: Inspect layout at required viewports**

Open the page in a browser automation session at 640×480, 800×480, and 1280×720. At each viewport verify:

- `document.documentElement.scrollWidth <= window.innerWidth`
- Map/canvas has non-zero width and height.
- 640×480 and 800×480 show `#compact-controls`; 1280×720 hides it.
- Opening the control drawer sets only `#left-panel.drawer-open`.
- Opening the waypoint drawer closes the left drawer and sets only `#right-panel.drawer-open`.
- Backdrop click and Escape close both drawers.
- Compact buttons have computed height of at least 40 pixels.

- [ ] **Step 3: Run diff hygiene checks**

Run:

```powershell
git diff --check -- gcs.html tests/test_gcs_bridge.py
git diff --stat -- gcs.html tests/test_gcs_bridge.py
```

Expected: no whitespace errors and only the two intended files are listed.

### Task 5: Commit and push the completed GCS work

**Files:**
- Commit: `gcs_bridge.py`
- Commit: `gcs.html`
- Commit: `tests/test_gcs_bridge.py`
- Existing design/plan commits remain in history.

- [ ] **Step 1: Re-run final verification from the repository root**

Run the complete test, Python syntax, JavaScript syntax, and `git diff --check` commands from Tasks 3 and 4. Expected: zero failures.

- [ ] **Step 2: Stage only ground-station files**

```powershell
git add -- gcs_bridge.py gcs.html tests/test_gcs_bridge.py docs/superpowers/plans/2026-07-22-raspberry-pi-640x480-gcs.md
git diff --cached --name-only
```

Expected staged paths are exactly those four files. Firmware binaries, build directories, archives, and unrelated untracked files are excluded.

- [ ] **Step 3: Commit implementation**

```powershell
git commit -m "feat: adapt GCS for 640x480 Raspberry Pi display"
```

- [ ] **Step 4: Push current branch**

```powershell
git push origin main
```

Expected: GitHub accepts the new design, plan, reviewed backend fixes, responsive frontend, and regression tests on `origin/main`.
