// ============================================
// Docs Server — manifest loader and safe file reader
// ============================================
//
// Backs the in-app documentation viewer. Two responsibilities:
//
//   1. Load and serve docs/manifest.json (the navigation tree).
//   2. Read markdown files from a strict whitelist of locations.
//
// Path validation is the security-critical bit. Every file request goes
// through resolveDoc() which:
//   - rejects null bytes
//   - resolves the path (flattens .. and absolute paths)
//   - follows symlinks via realpath
//   - enforces a .md extension
//   - requires the resolved path to fall under one of ALLOWED_ROOTS or
//     to exactly equal one of ALLOWED_FILES
//
// If you want to expose a new doc location, add it to the whitelist
// below — do not weaken the validation logic.
//

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

// Repo root: server/src/web/ → ../../..
const REPO_ROOT = path.resolve(__dirname, '..', '..', '..');

// Directories that may contain markdown docs (recursive).
const ALLOWED_ROOTS = [
    path.join(REPO_ROOT, 'docs'),
    path.join(REPO_ROOT, 'localization'),
    path.join(REPO_ROOT, 'hardware'),
];

// Specific files outside the allowed roots that may also be served.
// Use this for top-level READMEs (and similar) that we want to expose
// without whitelisting their entire parent directory.
const ALLOWED_FILES = [
    path.join(REPO_ROOT, 'firmware', 'esp32-omni', 'README.md'),
    path.join(REPO_ROOT, 'firmware', 'esp32-omni', 'FIRMWARE_SUMMARY.md'),
];

const MANIFEST_PATH = path.join(REPO_ROOT, 'docs', 'manifest.json');

let manifestCache = null;
let manifestMtime = 0;

/**
 * Load (or reload) the manifest. Cached by mtime so we re-read when the
 * file changes on disk without needing a server restart.
 */
export function getManifest() {
    let stat;
    try {
        stat = fs.statSync(MANIFEST_PATH);
    } catch {
        throw new Error('manifest not found');
    }

    if (manifestCache && stat.mtimeMs === manifestMtime) {
        return manifestCache;
    }

    const raw = fs.readFileSync(MANIFEST_PATH, 'utf8');
    const parsed = JSON.parse(raw);
    manifestCache = parsed;
    manifestMtime = stat.mtimeMs;
    return parsed;
}

/**
 * Resolve a relative doc path against REPO_ROOT and validate it.
 * Returns the absolute, real (symlink-resolved) path on success.
 * Throws on any rule violation.
 */
export function resolveDoc(rel) {
    if (typeof rel !== 'string' || rel.length === 0) {
        throw new Error('path required');
    }
    if (rel.includes('\0')) {
        throw new Error('null byte in path');
    }

    // Resolve against repo root, flattening any .. segments.
    const abs = path.resolve(REPO_ROOT, rel);

    // Follow symlinks to defeat symlink-out-of-root tricks.
    let real;
    try {
        real = fs.realpathSync(abs);
    } catch {
        throw new Error('not found');
    }

    // Markdown only.
    if (path.extname(real).toLowerCase() !== '.md') {
        throw new Error('not markdown');
    }

    // Must be in an allowed root or exactly equal an allowed file.
    const inAllowedRoot = ALLOWED_ROOTS.some((root) => {
        const realRoot = fs.realpathSync(root);
        return real === realRoot || real.startsWith(realRoot + path.sep);
    });
    const isAllowedFile = ALLOWED_FILES.some((file) => {
        try {
            return real === fs.realpathSync(file);
        } catch {
            return false;
        }
    });

    if (!inAllowedRoot && !isAllowedFile) {
        throw new Error('outside allowed roots');
    }

    return real;
}

/**
 * Read and return the content of a doc by its repo-relative path.
 * Throws on validation failure.
 */
export function readDoc(rel) {
    const real = resolveDoc(rel);
    return fs.readFileSync(real, 'utf8');
}

// Source-file extensions allowed for the citation-preview endpoint.
// Markdown is excluded here so the source endpoint can't be used as an
// alternate path to read docs (use readDoc for that).
const ALLOWED_SOURCE_EXTS = new Set([
    '.cpp', '.h', '.hpp', '.c', '.cc',
    '.js', '.mjs', '.cjs', '.ts',
    '.json', '.ini', '.cfg', '.toml', '.yaml', '.yml',
    '.py',
    '.html', '.css',
]);

// Roots that source files may live in. Note that docs/, localization/,
// and hardware/ are intentionally NOT here — those are doc territory.
const ALLOWED_SOURCE_ROOTS = [
    path.join(REPO_ROOT, 'firmware'),
    path.join(REPO_ROOT, 'server', 'src'),
    path.join(REPO_ROOT, 'server', 'tests'),
    path.join(REPO_ROOT, 'evaluation'),
];

/**
 * Resolve and validate a source-file path for the citation preview
 * endpoint. Same security model as resolveDoc, with a different
 * extension whitelist and root list.
 */
export function resolveSource(rel) {
    if (typeof rel !== 'string' || rel.length === 0) {
        throw new Error('path required');
    }
    if (rel.includes('\0')) {
        throw new Error('null byte in path');
    }

    const abs = path.resolve(REPO_ROOT, rel);
    let real;
    try {
        real = fs.realpathSync(abs);
    } catch {
        throw new Error('not found');
    }

    if (!ALLOWED_SOURCE_EXTS.has(path.extname(real).toLowerCase())) {
        throw new Error('extension not allowed');
    }

    const inAllowedRoot = ALLOWED_SOURCE_ROOTS.some((root) => {
        try {
            const realRoot = fs.realpathSync(root);
            return real === realRoot || real.startsWith(realRoot + path.sep);
        } catch {
            return false;
        }
    });

    if (!inAllowedRoot) {
        throw new Error('outside allowed roots');
    }

    return real;
}

/**
 * Read a slice of a source file (1-indexed inclusive line range).
 * `start` and `end` may be undefined for the full file. The slice is
 * capped at MAX_SOURCE_LINES to prevent abusive requests.
 */
const MAX_SOURCE_LINES = 500;

export function readSourceSlice(rel, start, end) {
    const real = resolveSource(rel);
    const text = fs.readFileSync(real, 'utf8');
    const lines = text.split('\n');

    const s = Number.isFinite(start) ? Math.max(1, Math.floor(start)) : 1;
    let e = Number.isFinite(end) ? Math.min(lines.length, Math.floor(end)) : lines.length;
    if (e < s) e = s;
    if (e - s + 1 > MAX_SOURCE_LINES) {
        e = s + MAX_SOURCE_LINES - 1;
    }

    return {
        path: rel,
        start: s,
        end: e,
        totalLines: lines.length,
        content: lines.slice(s - 1, e).join('\n'),
    };
}

// ============================================
// Image serving (for docs viewer)
// ============================================
//
// Same security model as resolveDoc, with an image-extension whitelist
// and its own root list. We allow images from docs/images/ (the
// curated source) and docs/images/ (the existing
// robot photos in the LaTeX paper directory).

const ALLOWED_IMAGE_EXTS = new Set([
    '.png', '.jpg', '.jpeg', '.gif', '.webp', '.svg',
]);

const ALLOWED_IMAGE_ROOTS = [
    path.join(REPO_ROOT, 'docs', 'images'),
    path.join(REPO_ROOT, 'publications', 'icefront-2026', 'figures'),
    path.join(REPO_ROOT, 'publications', 'icefront-2026', 'images'),
    path.join(REPO_ROOT, 'hardware'),
];

const IMAGE_MIME = {
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.gif': 'image/gif',
    '.webp': 'image/webp',
    '.svg': 'image/svg+xml',
};

export function resolveImage(rel) {
    if (typeof rel !== 'string' || rel.length === 0) {
        throw new Error('path required');
    }
    if (rel.includes('\0')) {
        throw new Error('null byte in path');
    }

    const abs = path.resolve(REPO_ROOT, rel);
    let real;
    try {
        real = fs.realpathSync(abs);
    } catch {
        throw new Error('not found');
    }

    const ext = path.extname(real).toLowerCase();
    if (!ALLOWED_IMAGE_EXTS.has(ext)) {
        throw new Error('extension not allowed');
    }

    const inAllowedRoot = ALLOWED_IMAGE_ROOTS.some((root) => {
        try {
            const realRoot = fs.realpathSync(root);
            return real === realRoot || real.startsWith(realRoot + path.sep);
        } catch {
            return false;
        }
    });

    if (!inAllowedRoot) {
        throw new Error('outside allowed roots');
    }

    return real;
}

export function readImage(rel) {
    const real = resolveImage(rel);
    const ext = path.extname(real).toLowerCase();
    return {
        path: real,
        mime: IMAGE_MIME[ext] || 'application/octet-stream',
        data: fs.readFileSync(real),
    };
}

// Test/debug exports
export const _internal = {
    REPO_ROOT,
    ALLOWED_ROOTS,
    ALLOWED_FILES,
    ALLOWED_SOURCE_ROOTS,
    ALLOWED_SOURCE_EXTS,
    ALLOWED_IMAGE_ROOTS,
    ALLOWED_IMAGE_EXTS,
    MAX_SOURCE_LINES,
};
