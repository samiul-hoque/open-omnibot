// ============================================
// Theme — light/dark mode controller
// ============================================
//
// Reads/writes the `data-theme` attribute on <html>, persists choice in
// localStorage, and notifies subscribers (canvas widgets) on change so they
// can re-read CSS tokens and redraw.
//
// Call Theme.init() inline in <head> *before* aegis.css loads to avoid a
// flash of the wrong theme on reload.

(function (global) {
    'use strict';

    const STORAGE_KEY = 'omni-theme';
    const TOKEN_NAMES = [
        '--ak-bg-deep', '--ak-surface-l1', '--ak-surface-l2',
        '--ak-accent', '--ak-accent-dim',
        '--ak-success', '--ak-warning', '--ak-error',
        '--ak-text', '--ak-text-muted', '--ak-data', '--ak-border',
        '--ak-track', '--ak-grid-small', '--ak-grid-large',
    ];

    const listeners = new Set();

    function readTokens() {
        const cs = getComputedStyle(document.documentElement);
        const out = {};
        for (const name of TOKEN_NAMES) {
            // Strip the "--ak-" prefix and convert kebab-case to camelCase.
            const key = name.replace('--ak-', '').replace(/-([a-z])/g, (_, c) => c.toUpperCase());
            out[key] = cs.getPropertyValue(name).trim();
        }
        return out;
    }

    global.Theme = {
        current() {
            return document.documentElement.getAttribute('data-theme') || 'dark';
        },

        tokens() {
            return readTokens();
        },

        set(name) {
            const next = name === 'light' ? 'light' : 'dark';
            document.documentElement.setAttribute('data-theme', next);
            try { localStorage.setItem(STORAGE_KEY, next); } catch { /* ignore */ }
            listeners.forEach(fn => {
                try { fn(next); } catch (err) { console.error(err); }
            });
        },

        toggle() {
            this.set(this.current() === 'light' ? 'dark' : 'light');
        },

        onChange(fn) {
            listeners.add(fn);
            return () => listeners.delete(fn);
        },

        init() {
            let saved = null;
            try { saved = localStorage.getItem(STORAGE_KEY); } catch { /* ignore */ }
            const prefersLight = global.matchMedia && global.matchMedia('(prefers-color-scheme: light)').matches;
            const initial = saved || (prefersLight ? 'light' : 'dark');
            document.documentElement.setAttribute('data-theme', initial);
        },
    };
})(typeof window !== 'undefined' ? window : this);
