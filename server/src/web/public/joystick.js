// ============================================
// Virtual Joystick Component
// ============================================
//
// Canvas-based joystick supporting mouse and touch input.
// Returns normalized x, y values from -1 to 1.
//

/* global Theme */

export class Joystick {
    constructor(canvas, options = {}) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');

        // Options
        this.onMove = options.onMove || (() => {});
        this.onRelease = options.onRelease || (() => {});

        // Dimensions
        this.centerX = canvas.width / 2;
        this.centerY = canvas.height / 2;
        this.baseRadius = Math.min(canvas.width, canvas.height) / 2 - 10;
        this.stickRadius = this.baseRadius * 0.4;
        this.maxDistance = this.baseRadius - this.stickRadius;

        // State
        this.stickX = this.centerX;
        this.stickY = this.centerY;
        this.active = false;
        this.touchId = null;

        // Throttling
        this.lastEmit = 0;
        this.emitInterval = 100; // 10Hz max

        // Colors — populated from theme tokens; refreshed on theme change
        this.colors = {};
        this.refreshColors();

        if (Theme && typeof Theme.onChange === 'function') {
            this._unsubTheme = Theme.onChange(() => {
                this.refreshColors();
                this.draw();
            });
        }

        // Bind event handlers
        this.handleMouseDown = this.handleMouseDown.bind(this);
        this.handleMouseMove = this.handleMouseMove.bind(this);
        this.handleMouseUp = this.handleMouseUp.bind(this);
        this.handleTouchStart = this.handleTouchStart.bind(this);
        this.handleTouchMove = this.handleTouchMove.bind(this);
        this.handleTouchEnd = this.handleTouchEnd.bind(this);

        // Attach events
        canvas.addEventListener('mousedown', this.handleMouseDown);
        document.addEventListener('mousemove', this.handleMouseMove);
        document.addEventListener('mouseup', this.handleMouseUp);
        canvas.addEventListener('touchstart', this.handleTouchStart, { passive: false });
        document.addEventListener('touchmove', this.handleTouchMove, { passive: false });
        document.addEventListener('touchend', this.handleTouchEnd);
        document.addEventListener('touchcancel', this.handleTouchEnd);

        // Initial draw
        this.draw();
    }

    refreshColors() {
        const t = (Theme && Theme.tokens) ? Theme.tokens() : {};
        const get = (key, fallback) => t[key] || fallback;
        this.colors = {
            base:        get('surfaceL2', '#0f3460'),
            baseStroke:  get('accent',    '#00d4ff'),
            stick:       get('accent',    '#00d4ff'),
            stickActive: get('success',   '#00ff88'),
            crosshair:   get('border',    'rgba(0, 212, 255, 0.3)'),
        };
    }

    getCanvasPosition(clientX, clientY) {
        const rect = this.canvas.getBoundingClientRect();
        return {
            x: clientX - rect.left,
            y: clientY - rect.top,
        };
    }

    handleMouseDown(e) {
        e.preventDefault();
        this.active = true;
        const pos = this.getCanvasPosition(e.clientX, e.clientY);
        this.updateStickPosition(pos.x, pos.y);
    }

    handleMouseMove(e) {
        if (!this.active) return;
        const pos = this.getCanvasPosition(e.clientX, e.clientY);
        this.updateStickPosition(pos.x, pos.y);
    }

    handleMouseUp() {
        if (!this.active) return;
        this.active = false;
        this.resetStick();
    }

    handleTouchStart(e) {
        e.preventDefault();
        if (this.touchId !== null) return;

        const touch = e.changedTouches[0];
        this.touchId = touch.identifier;
        this.active = true;

        const pos = this.getCanvasPosition(touch.clientX, touch.clientY);
        this.updateStickPosition(pos.x, pos.y);
    }

    handleTouchMove(e) {
        if (this.touchId === null) return;

        for (let i = 0; i < e.changedTouches.length; i++) {
            const touch = e.changedTouches[i];
            if (touch.identifier === this.touchId) {
                e.preventDefault();
                const pos = this.getCanvasPosition(touch.clientX, touch.clientY);
                this.updateStickPosition(pos.x, pos.y);
                break;
            }
        }
    }

    handleTouchEnd(e) {
        for (let i = 0; i < e.changedTouches.length; i++) {
            if (e.changedTouches[i].identifier === this.touchId) {
                this.touchId = null;
                this.active = false;
                this.resetStick();
                break;
            }
        }
    }

    updateStickPosition(x, y) {
        // Calculate distance from center
        let dx = x - this.centerX;
        let dy = y - this.centerY;
        const distance = Math.sqrt(dx * dx + dy * dy);

        // Clamp to max distance
        if (distance > this.maxDistance) {
            const scale = this.maxDistance / distance;
            dx *= scale;
            dy *= scale;
        }

        this.stickX = this.centerX + dx;
        this.stickY = this.centerY + dy;

        this.draw();
        this.emitMove();
    }

    resetStick() {
        this.stickX = this.centerX;
        this.stickY = this.centerY;
        this.draw();
        this.onRelease();
    }

    emitMove() {
        const now = Date.now();
        if (now - this.lastEmit < this.emitInterval) return;
        this.lastEmit = now;

        // Normalize to -1 to 1
        const normalizedX = (this.stickX - this.centerX) / this.maxDistance;
        const normalizedY = -(this.stickY - this.centerY) / this.maxDistance; // Invert Y

        this.onMove(normalizedX, normalizedY);
    }

    draw() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // Clear canvas
        ctx.clearRect(0, 0, width, height);

        // Draw base circle
        ctx.beginPath();
        ctx.arc(this.centerX, this.centerY, this.baseRadius, 0, Math.PI * 2);
        ctx.fillStyle = this.colors.base;
        ctx.fill();
        ctx.strokeStyle = this.colors.baseStroke;
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw crosshair lines
        ctx.beginPath();
        ctx.moveTo(this.centerX, this.centerY - this.baseRadius + 10);
        ctx.lineTo(this.centerX, this.centerY + this.baseRadius - 10);
        ctx.strokeStyle = this.colors.crosshair;
        ctx.lineWidth = 1;
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(this.centerX - this.baseRadius + 10, this.centerY);
        ctx.lineTo(this.centerX + this.baseRadius - 10, this.centerY);
        ctx.stroke();

        // Draw stick
        ctx.beginPath();
        ctx.arc(this.stickX, this.stickY, this.stickRadius, 0, Math.PI * 2);
        ctx.fillStyle = this.active ? this.colors.stickActive : this.colors.stick;
        ctx.fill();

        // Draw stick highlight
        const gradient = ctx.createRadialGradient(
            this.stickX - this.stickRadius * 0.3,
            this.stickY - this.stickRadius * 0.3,
            0,
            this.stickX,
            this.stickY,
            this.stickRadius,
        );
        gradient.addColorStop(0, 'rgba(255, 255, 255, 0.3)');
        gradient.addColorStop(1, 'rgba(255, 255, 255, 0)');
        ctx.beginPath();
        ctx.arc(this.stickX, this.stickY, this.stickRadius, 0, Math.PI * 2);
        ctx.fillStyle = gradient;
        ctx.fill();
    }

    destroy() {
        this.canvas.removeEventListener('mousedown', this.handleMouseDown);
        document.removeEventListener('mousemove', this.handleMouseMove);
        document.removeEventListener('mouseup', this.handleMouseUp);
        this.canvas.removeEventListener('touchstart', this.handleTouchStart);
        document.removeEventListener('touchmove', this.handleTouchMove);
        document.removeEventListener('touchend', this.handleTouchEnd);
        document.removeEventListener('touchcancel', this.handleTouchEnd);
        if (this._unsubTheme) {
            this._unsubTheme();
            this._unsubTheme = null;
        }
    }
}
