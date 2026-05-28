// ============================================
// 2D Robot Map Component
// ============================================
//
// Canvas-based 2D map showing robot position trail,
// grid lines, and heading arrow.
//

/* global Theme */

export class RobotMap {
    constructor(canvas, options = {}) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');

        // Configuration
        this.maxTrailPoints = options.maxTrailPoints || 5000;
        this.gridSmall = options.gridSmall || 0.1;  // 10cm
        this.gridLarge = options.gridLarge || 0.5;  // 50cm
        this.robotSize = options.robotSize || 0.15; // Robot visual size in meters

        // Multi-layer trails — `color` is set by refreshColors() from theme tokens
        this.layers = {
            deadReckoning:  { trail: [], color: '', label: 'Dead Reckoning',  visible: true,  lineWidth: 2 },
            imuFusion:      { trail: [], color: '', label: 'IMU Fusion',      visible: false, lineWidth: 2 },
        };

        // Backward-compatible alias
        Object.defineProperty(this, 'trail', {
            get() { return this.layers.deadReckoning.trail; },
            set(v) { this.layers.deadReckoning.trail = v; },
        });

        // State
        this.currentPose = { x: 0, y: 0, theta: 0 };
        this._drawScheduled = false;
        this._legendWidthCache = null;

        // Expected start pose (dashed ghost-robot) shown during the armed
        // state. Populated by experiment_armed via setStartHint({x, y,
        // headingDeg}); cleared on state transition away from armed. Helps
        // the operator visually line up the physical robot with the
        // trajectory's intended origin pose — the dashboard map only tracks
        // odometry (body-frame from arm time), so without this hint the
        // robot icon snaps to (0,0) on every arm regardless of where the
        // trajectory is actually supposed to start in world frame.
        this.startHint = null;

        // View state (will be calculated in initializeView)
        this.scale = 100; // pixels per meter (adjustable)
        this.offsetX = 0;
        this.offsetY = 0;
        this.autoCenter = false; // Start with grid view, not robot-centered

        // Colors (populated from theme tokens; refreshed on theme change)
        this.colors = {};
        this.refreshColors();

        if (Theme && typeof Theme.onChange === 'function') {
            this._unsubTheme = Theme.onChange(() => {
                this.refreshColors();
                this.draw();
            });
        }

        // Grid bounds in world coordinates (meters). Origin (0,0) is rendered
        // with a crosshair; the bounds define the usable camera workspace.
        // Camera-frame convention (post-GT-rig refactor): origin centred,
        // X is the long axis (±1.35 m usable), Y is the short axis
        // (±0.55 m usable). Matches the handover's measured usable region.
        this.gridBounds = { xMin: -1.35, xMax: 1.35, yMin: -0.55, yMax: 0.55 };

        // Initial setup
        this.handleResize();
        this.initializeView();

        // Handle resize
        this.resizeObserver = new ResizeObserver(() => this.handleResize());
        this.resizeObserver.observe(canvas.parentElement);

        // Mouse wheel zoom
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            if (e.deltaY < 0) {
                this.zoomIn();
            } else {
                this.zoomOut();
            }
        }, { passive: false });
    }

    refreshColors() {
        const t = (Theme && Theme.tokens) ? Theme.tokens() : {};
        // Fallback to dark-mode literals if Theme isn't loaded.
        const get = (key, fallback) => t[key] || fallback;
        const isLight = (Theme && Theme.current && Theme.current() === 'light');

        this.colors = {
            background:   get('bgDeep',     '#0b1326'),
            gridSmall:    get('gridSmall',  'rgba(59,130,246,0.08)'),
            gridLarge:    get('gridLarge',  'rgba(59,130,246,0.20)'),
            trail:        get('accent',     '#3B82F6'),
            robot:        get('success',    '#10b981'),
            robotOutline: get('text',       '#dae2fd'),
            heading:      get('error',      '#ef4444'),
            origin:       get('error',      '#ef4444'),
            text:         get('text',       '#dae2fd'),
            textMuted:    get('textMuted',  'rgba(218,226,253,0.5)'),
            surface:      get('surfaceL1',  '#131b2e'),
            border:       get('border',     'rgba(59,130,246,0.15)'),
            // Translucent overlays for in-canvas widgets (compass, legend).
            overlayBg:     isLight ? 'rgba(255,255,255,0.78)' : 'rgba(0,0,0,0.5)',
            overlayBorder: isLight ? 'rgba(15,23,42,0.20)'    : 'rgba(255,255,255,0.3)',
            // Compass axis identity colors — adjusted per theme for contrast.
            axisX:         isLight ? '#dc2626' : '#ff4444',
            axisY:         isLight ? '#16a34a' : '#44ff44',
        };

        // Layer trail colors — keep semantic identity per theme token.
        this.layers.deadReckoning.color = get('warning', '#f59e0b');
        this.layers.imuFusion.color     = get('success', '#10b981');
    }

    handleResize() {
        const parent = this.canvas.parentElement;
        // Match the canvas pixel buffer to the container's CSS size so the
        // browser doesn't stretch a square buffer into a non-square box.
        const width = parent.clientWidth;
        const height = parent.clientHeight;
        if (width > 0 && height > 0) {
            this.canvas.width = width;
            this.canvas.height = height;
            this.draw();
        }
    }

    // Initialize view to fit the full grid
    initializeView() {
        const { xMin, xMax, yMin, yMax } = this.gridBounds;
        const gridW = xMax - xMin;
        const gridH = yMax - yMin;
        const canvasSize = Math.min(this.canvas.width, this.canvas.height);

        if (canvasSize <= 0 || gridW <= 0 || gridH <= 0) {
            return;
        }

        // Calculate scale to fit grid with 15% padding on each side (70% of canvas for grid)
        const padding = 0.15;
        const availableSize = canvasSize * (1 - 2 * padding);

        const scaleX = availableSize / gridW;
        const scaleY = availableSize / gridH;
        this.scale = Math.min(scaleX, scaleY);

        // Center view on the geometric center of the grid (not the origin)
        this.offsetX = (xMin + xMax) / 2;
        this.offsetY = (yMin + yMax) / 2;

        this.draw();
    }

    updatePose(x, y, theta) {
        // Add to trail if position changed significantly
        const lastPoint = this.trail[this.trail.length - 1];
        if (!lastPoint ||
                Math.abs(x - lastPoint.x) > 0.005 ||
                Math.abs(y - lastPoint.y) > 0.005) {
            this.trail.push({ x, y });

            // Trim trail if too long (splice in place to avoid array re-allocation)
            if (this.trail.length > this.maxTrailPoints) {
                this.trail.splice(0, this.trail.length - this.maxTrailPoints);
            }
        }

        this.currentPose = { x, y, theta };

        // Auto-center on robot
        if (this.autoCenter) {
            this.centerOnRobot();
        }

        this.scheduleDraw();
    }

    clearTrail() {
        this.layers.deadReckoning.trail = [];
        this.draw();
    }

    setStartHint(hint) {
        // Accept { x, y, headingDeg } from the trajectory catalog. Any
        // missing/non-finite field disables the ghost; that makes the
        // method tolerant of partial hints (e.g. a text-only startHint
        // without numeric coordinates).
        if (!hint ||
            !Number.isFinite(hint.x) ||
            !Number.isFinite(hint.y) ||
            !Number.isFinite(hint.headingDeg)) {
            this.startHint = null;
        } else {
            this.startHint = {
                x: hint.x,
                y: hint.y,
                theta: hint.headingDeg * Math.PI / 180,
            };
        }
        this.scheduleDraw();
    }

    clearStartHint() {
        this.startHint = null;
        this.scheduleDraw();
    }

    // --- Layer Management ---

    updateLayerPose(layerName, x, y) {
        const layer = this.layers[layerName];
        if (!layer) return;
        const last = layer.trail[layer.trail.length - 1];
        if (!last || Math.abs(x - last.x) > 0.005 || Math.abs(y - last.y) > 0.005) {
            layer.trail.push({ x, y });
            if (layer.trail.length > this.maxTrailPoints) {
                layer.trail.splice(0, layer.trail.length - this.maxTrailPoints);
            }
        }
    }

    setLayerVisible(layerName, visible) {
        const layer = this.layers[layerName];
        if (layer) {
            layer.visible = visible;
            this.draw();
        }
    }

    clearLayerTrail(layerName) {
        const layer = this.layers[layerName];
        if (layer) { layer.trail = []; this.draw(); }
    }

    clearAllTrails() {
        for (const layer of Object.values(this.layers)) layer.trail = [];
        this.draw();
    }

    centerOnRobot() {
        this.offsetX = this.currentPose.x;
        this.offsetY = this.currentPose.y;
    }

    setAutoCenter(enabled) {
        this.autoCenter = enabled;
        if (enabled) {
            this.centerOnRobot();
            this.draw();
        } else {
            // When disabling auto-center, show full grid view
            this.initializeView();
        }
    }

    setScale(scale) {
        this.scale = Math.max(20, Math.min(500, scale));
        this.draw();
    }

    zoomIn() {
        this.setScale(this.scale * 1.5);
    }

    zoomOut() {
        this.setScale(this.scale / 1.5);
    }

    // Convert world coordinates to canvas coordinates
    worldToCanvas(wx, wy) {
        const cx = this.canvas.width / 2 + (wx - this.offsetX) * this.scale;
        const cy = this.canvas.height / 2 - (wy - this.offsetY) * this.scale; // Flip Y
        return { x: cx, y: cy };
    }

    // Convert canvas coordinates to world coordinates
    canvasToWorld(cx, cy) {
        const wx = (cx - this.canvas.width / 2) / this.scale + this.offsetX;
        const wy = -(cy - this.canvas.height / 2) / this.scale + this.offsetY;
        return { x: wx, y: wy };
    }

    // Set explicit grid bounds in world coordinates and re-fit the view.
    // The grid is drawn as [xMin, xMax] × [yMin, yMax] with the origin (0,0)
    // marked wherever it falls inside.
    setGridBounds(xMin, xMax, yMin, yMax) {
        this.gridBounds = { xMin, xMax, yMin, yMax };
        this.initializeView();
    }

    // Backward-compatible shim: interpret (width, height) as bounds flush with
    // the left edge (x ∈ [0, width]) and vertically centered (y ∈ [-h/2, +h/2]).
    setGridSize(width, height) {
        this.setGridBounds(0, width, -height / 2, height / 2);
    }

    getGridBounds() {
        return { ...this.gridBounds };
    }

    getGridSize() {
        const { xMin, xMax, yMin, yMax } = this.gridBounds;
        return { width: xMax - xMin, height: yMax - yMin };
    }

    scheduleDraw() {
        if (this._drawScheduled) return;
        this._drawScheduled = true;
        requestAnimationFrame(() => {
            this._drawScheduled = false;
            this.draw();
        });
    }

    draw() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // Clear canvas
        ctx.fillStyle = this.colors.background;
        ctx.fillRect(0, 0, width, height);

        // Calculate visible world bounds
        const topLeft = this.canvasToWorld(0, 0);
        const bottomRight = this.canvasToWorld(width, height);

        // Draw grid
        this.drawGrid(topLeft, bottomRight);

        // Draw physical grid boundary (camera usable workspace outline)
        this.drawGridBoundary();

        // Draw origin marker
        this.drawOrigin();

        // Draw trail
        this.drawTrail();

        // Draw expected start pose (armed only — setStartHint/clearStartHint
        // wired to experiment_armed / state transitions)
        if (this.startHint) this.drawStartHint();

        // Draw robot
        this.drawRobot();

        // Draw scale indicator
        this.drawScaleIndicator();

        // Draw compass (direction indicator)
        this.drawCompass();

        // Draw legend
        this.drawLegend();
    }

    drawGrid(topLeft, bottomRight) {
        const ctx = this.ctx;

        // Draw small grid lines
        ctx.strokeStyle = this.colors.gridSmall;
        ctx.lineWidth = 1;

        const startX = Math.floor(topLeft.x / this.gridSmall) * this.gridSmall;
        const endX = Math.ceil(bottomRight.x / this.gridSmall) * this.gridSmall;
        const startY = Math.floor(bottomRight.y / this.gridSmall) * this.gridSmall;
        const endY = Math.ceil(topLeft.y / this.gridSmall) * this.gridSmall;

        // Vertical lines
        for (let x = startX; x <= endX; x += this.gridSmall) {
            if (Math.abs(x % this.gridLarge) < 0.001) continue; // Skip large grid lines
            const start = this.worldToCanvas(x, topLeft.y);
            const end = this.worldToCanvas(x, bottomRight.y);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }

        // Horizontal lines
        for (let y = startY; y <= endY; y += this.gridSmall) {
            if (Math.abs(y % this.gridLarge) < 0.001) continue;
            const start = this.worldToCanvas(topLeft.x, y);
            const end = this.worldToCanvas(bottomRight.x, y);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }

        // Draw large grid lines
        ctx.strokeStyle = this.colors.gridLarge;
        ctx.lineWidth = 1;

        const startXL = Math.floor(topLeft.x / this.gridLarge) * this.gridLarge;
        const endXL = Math.ceil(bottomRight.x / this.gridLarge) * this.gridLarge;
        const startYL = Math.floor(bottomRight.y / this.gridLarge) * this.gridLarge;
        const endYL = Math.ceil(topLeft.y / this.gridLarge) * this.gridLarge;

        for (let x = startXL; x <= endXL; x += this.gridLarge) {
            const start = this.worldToCanvas(x, topLeft.y);
            const end = this.worldToCanvas(x, bottomRight.y);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }

        for (let y = startYL; y <= endYL; y += this.gridLarge) {
            const start = this.worldToCanvas(topLeft.x, y);
            const end = this.worldToCanvas(bottomRight.x, y);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }
    }

    drawGridBoundary() {
        const ctx = this.ctx;
        const { xMin, xMax, yMin, yMax } = this.gridBounds;
        const tl = this.worldToCanvas(xMin, yMax);
        const br = this.worldToCanvas(xMax, yMin);

        ctx.strokeStyle = this.colors.gridLarge;
        ctx.lineWidth = 2;
        ctx.setLineDash([]);
        ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);
    }

    drawOrigin() {
        const ctx = this.ctx;
        const origin = this.worldToCanvas(0, 0);

        // Draw origin crosshair
        ctx.strokeStyle = this.colors.origin;
        ctx.lineWidth = 2;

        const size = 15;
        ctx.beginPath();
        ctx.moveTo(origin.x - size, origin.y);
        ctx.lineTo(origin.x + size, origin.y);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y - size);
        ctx.lineTo(origin.x, origin.y + size);
        ctx.stroke();

        // Draw origin circle
        ctx.beginPath();
        ctx.arc(origin.x, origin.y, 5, 0, Math.PI * 2);
        ctx.fillStyle = this.colors.origin;
        ctx.fill();
    }

    drawTrail() {
        const ctx = this.ctx;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';

        for (const layer of Object.values(this.layers)) {
            if (!layer.visible || layer.trail.length < 2) continue;

            ctx.lineWidth = layer.lineWidth || 2;
            if (layer.dash) ctx.setLineDash(layer.dash);
            else ctx.setLineDash([]);

            // Parse color once outside the loop for alpha gradient
            const color = layer.color;
            const cr = parseInt(color.slice(1, 3), 16);
            const cg = parseInt(color.slice(3, 5), 16);
            const cb = parseInt(color.slice(5, 7), 16);
            for (let i = 1; i < layer.trail.length; i++) {
                const alpha = 0.15 + 0.85 * (i / layer.trail.length);
                ctx.strokeStyle = `rgba(${cr},${cg},${cb},${alpha})`;

                const prev = this.worldToCanvas(layer.trail[i - 1].x, layer.trail[i - 1].y);
                const curr = this.worldToCanvas(layer.trail[i].x, layer.trail[i].y);

                ctx.beginPath();
                ctx.moveTo(prev.x, prev.y);
                ctx.lineTo(curr.x, curr.y);
                ctx.stroke();
            }
        }
        ctx.setLineDash([]);
    }

    drawStartHint() {
        const ctx = this.ctx;
        const pos = this.worldToCanvas(this.startHint.x, this.startHint.y);
        const size = this.robotSize * this.scale;
        const half = size / 2;

        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-this.startHint.theta);

        // Dashed outline, accent colour, semi-transparent — visually
        // distinct from the solid robot icon without needing a new colour
        // in the theme palette.
        ctx.strokeStyle = this.colors.heading || '#3b82f6';
        ctx.setLineDash([6, 4]);
        ctx.lineWidth = 2;
        ctx.globalAlpha = 0.75;
        ctx.strokeRect(-half, -half, size, size);

        // Heading arrow (same geometry as drawRobot for consistency).
        const arrowLen = size * 0.8;
        const arrowW = size * 0.3;
        ctx.beginPath();
        ctx.moveTo(half + 5, 0);
        ctx.lineTo(half - arrowLen * 0.3, -arrowW / 2);
        ctx.lineTo(half - arrowLen * 0.3,  arrowW / 2);
        ctx.closePath();
        ctx.stroke();

        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
        ctx.restore();

        // "Start" label anchored just above the ghost so it doesn't
        // overlap with the robot icon once preflight passes and the
        // actual robot icon moves into the same region.
        ctx.save();
        ctx.fillStyle = this.colors.heading || '#3b82f6';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillText('start', pos.x, pos.y - half - 4);
        ctx.restore();
    }

    drawRobot() {
        const ctx = this.ctx;
        const pos = this.worldToCanvas(this.currentPose.x, this.currentPose.y);
        const robotPixelSize = this.robotSize * this.scale;

        // Save context for rotation
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-this.currentPose.theta); // Negative because canvas Y is flipped

        // Draw robot body (square representing the chassis)
        const halfSize = robotPixelSize / 2;
        ctx.fillStyle = this.colors.robot;
        ctx.globalAlpha = 0.6;
        ctx.fillRect(-halfSize, -halfSize, robotPixelSize, robotPixelSize);
        ctx.globalAlpha = 1;

        // Draw robot outline
        ctx.strokeStyle = this.colors.robotOutline;
        ctx.lineWidth = 2;
        ctx.strokeRect(-halfSize, -halfSize, robotPixelSize, robotPixelSize);

        // Draw heading arrow (pointing in +X direction, which is forward)
        const arrowLength = robotPixelSize * 0.8;
        const arrowWidth = robotPixelSize * 0.3;

        ctx.fillStyle = this.colors.heading;
        ctx.beginPath();
        ctx.moveTo(halfSize + 5, 0); // Arrow tip
        ctx.lineTo(halfSize - arrowLength * 0.3, -arrowWidth / 2);
        ctx.lineTo(halfSize - arrowLength * 0.3, arrowWidth / 2);
        ctx.closePath();
        ctx.fill();

        // Draw center dot
        ctx.beginPath();
        ctx.arc(0, 0, 4, 0, Math.PI * 2);
        ctx.fillStyle = this.colors.robotOutline;
        ctx.fill();

        ctx.restore();
    }

    drawScaleIndicator() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // Calculate a nice round scale bar length
        let scaleBarMeters = 0.5;
        let scaleBarPixels = scaleBarMeters * this.scale;

        if (scaleBarPixels > 150) {
            scaleBarMeters = 0.2;
            scaleBarPixels = scaleBarMeters * this.scale;
        } else if (scaleBarPixels < 50) {
            scaleBarMeters = 1.0;
            scaleBarPixels = scaleBarMeters * this.scale;
        }

        // Draw scale bar in bottom-right corner
        const margin = 15;
        const barHeight = 4;
        const x = width - margin - scaleBarPixels;
        const y = height - margin;

        ctx.fillStyle = this.colors.text;
        ctx.fillRect(x, y - barHeight, scaleBarPixels, barHeight);

        // Draw end caps
        ctx.fillRect(x, y - barHeight - 3, 2, barHeight + 6);
        ctx.fillRect(x + scaleBarPixels - 2, y - barHeight - 3, 2, barHeight + 6);

        // Draw label
        ctx.fillStyle = this.colors.text;
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`${scaleBarMeters * 100}cm`, x + scaleBarPixels / 2, y - barHeight - 6);
    }

    drawCompass() {
        const ctx = this.ctx;
        const margin = 15;
        const size = 35;
        const cx = margin + size;
        const cy = margin + size;

        // Draw compass circle background
        ctx.beginPath();
        ctx.arc(cx, cy, size, 0, Math.PI * 2);
        ctx.fillStyle = this.colors.overlayBg;
        ctx.fill();
        ctx.strokeStyle = this.colors.overlayBorder;
        ctx.lineWidth = 1;
        ctx.stroke();

        // Draw X axis arrow (forward direction) - points right on screen = +X in world
        ctx.strokeStyle = this.colors.axisX;
        ctx.fillStyle = this.colors.axisX;
        ctx.lineWidth = 2;

        // Arrow shaft
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + size - 8, cy);
        ctx.stroke();

        // Arrow head
        ctx.beginPath();
        ctx.moveTo(cx + size - 5, cy);
        ctx.lineTo(cx + size - 12, cy - 5);
        ctx.lineTo(cx + size - 12, cy + 5);
        ctx.closePath();
        ctx.fill();

        // X label — placed inside the circle, just below the arrow tip
        ctx.font = 'bold 9px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = this.colors.axisX;
        ctx.fillText('X', cx + size - 14, cy + 9);

        // Draw Y axis arrow (left direction) - points up on screen = +Y in world
        ctx.strokeStyle = this.colors.axisY;
        ctx.fillStyle = this.colors.axisY;

        // Arrow shaft
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx, cy - size + 8);
        ctx.stroke();

        // Arrow head
        ctx.beginPath();
        ctx.moveTo(cx, cy - size + 5);
        ctx.lineTo(cx - 5, cy - size + 12);
        ctx.lineTo(cx + 5, cy - size + 12);
        ctx.closePath();
        ctx.fill();

        // Y label — placed inside the circle, just right of the arrow tip
        ctx.fillStyle = this.colors.axisY;
        ctx.fillText('Y', cx + 9, cy - size + 14);
        ctx.textBaseline = 'alphabetic';
    }

    drawLegend() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const margin = 10;
        const lineHeight = 14;
        const boxSize = 8;

        // Legend items — dynamic from visible layers + fixed items
        const items = [
            { color: this.colors.robot, label: 'Robot', shape: 'square' },
            { color: this.colors.heading, label: 'Heading', shape: 'arrow' },
        ];
        for (const layer of Object.values(this.layers)) {
            if (layer.visible) {
                items.push({ color: layer.color, label: layer.label, shape: 'line' });
            }
        }
        items.push({ color: this.colors.origin, label: 'Origin', shape: 'cross' });

        // Size the legend to the widest label so entries like "Dead Reckoning"
        // fit without clipping. 10 px sans-serif averages ≈5.5 px/char.
        ctx.font = '10px sans-serif';
        const labelPad = 8;                // space between swatch and text
        const swatchArea = 8 + labelPad;   // left pad + swatch + gap
        const rightPad = 10;
        // Cache legend width — only recompute when item count changes
        const cacheKey = items.length;
        if (!this._legendWidthCache || this._legendWidthCache.key !== cacheKey) {
            const maxLabelWidth = items.reduce((max, it) => {
                const w = ctx.measureText(it.label).width;
                return w > max ? w : max;
            }, 0);
            this._legendWidthCache = { key: cacheKey, width: maxLabelWidth };
        }
        const legendWidth = Math.ceil(swatchArea + this._legendWidthCache.width + rightPad);
        const legendHeight = items.length * lineHeight + 10;
        const x = width - margin - legendWidth;
        const y = margin;

        // Draw legend background
        ctx.fillStyle = this.colors.overlayBg;
        ctx.fillRect(x, y, legendWidth, legendHeight);
        ctx.strokeStyle = this.colors.overlayBorder;
        ctx.lineWidth = 1;
        ctx.strokeRect(x, y, legendWidth, legendHeight);

        // Draw each legend item
        ctx.textAlign = 'left';

        items.forEach((item, i) => {
            const itemX = x + 8;
            const itemY = y + 12 + i * lineHeight;

            ctx.fillStyle = item.color;
            ctx.strokeStyle = item.color;
            ctx.lineWidth = 2;

            // Draw shape
            switch (item.shape) {
            case 'square':
                ctx.fillRect(itemX, itemY - boxSize + 2, boxSize, boxSize);
                break;
            case 'circle':
                ctx.beginPath();
                ctx.arc(itemX + boxSize/2, itemY - boxSize/2 + 2, boxSize/2, 0, Math.PI * 2);
                ctx.fill();
                break;
            case 'diamond':
                ctx.beginPath();
                ctx.moveTo(itemX + boxSize/2, itemY - boxSize + 2);
                ctx.lineTo(itemX + boxSize, itemY - boxSize/2 + 2);
                ctx.lineTo(itemX + boxSize/2, itemY + 2);
                ctx.lineTo(itemX, itemY - boxSize/2 + 2);
                ctx.closePath();
                ctx.fill();
                break;
            case 'line':
                ctx.beginPath();
                ctx.moveTo(itemX, itemY - boxSize/2 + 2);
                ctx.lineTo(itemX + boxSize, itemY - boxSize/2 + 2);
                ctx.stroke();
                break;
            case 'arrow':
                ctx.beginPath();
                ctx.moveTo(itemX, itemY - boxSize/2 + 2);
                ctx.lineTo(itemX + boxSize, itemY - boxSize/2 + 2);
                ctx.lineTo(itemX + boxSize - 3, itemY - boxSize + 2);
                ctx.moveTo(itemX + boxSize, itemY - boxSize/2 + 2);
                ctx.lineTo(itemX + boxSize - 3, itemY + 2);
                ctx.stroke();
                break;
            case 'cross':
                ctx.beginPath();
                ctx.moveTo(itemX, itemY - boxSize/2 + 2);
                ctx.lineTo(itemX + boxSize, itemY - boxSize/2 + 2);
                ctx.moveTo(itemX + boxSize/2, itemY - boxSize + 2);
                ctx.lineTo(itemX + boxSize/2, itemY + 2);
                ctx.stroke();
                break;
            }

            // Draw label
            ctx.fillStyle = this.colors.text;
            ctx.fillText(item.label, itemX + boxSize + 6, itemY);
        });
    }

    destroy() {
        if (this.resizeObserver) {
            this.resizeObserver.disconnect();
        }
        if (this._unsubTheme) {
            this._unsubTheme();
            this._unsubTheme = null;
        }
    }
}
