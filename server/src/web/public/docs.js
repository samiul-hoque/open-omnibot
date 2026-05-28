// ============================================
// Docs Viewer — view controller
// ============================================
//
// Responsibilities:
//   1. Fetch /api/docs/manifest, build the sidebar tree.
//   2. Render the landing page (categories + summaries) when no doc is open.
//   3. Fetch and render markdown docs via marked + highlight.js.
//   4. Strip and display YAML frontmatter as a small header block.
//   5. Generate a right-side TOC from h2/h3 with scroll-spy.
//   6. Intercept relative .md links — route through the viewer.
//   7. Linkify "path/to/file.cpp:NN-MM" patterns into clickable previews
//      backed by /api/docs/source.
//   8. Theme-aware highlight.js stylesheet swap.
//   9. Hash routing (#docs/<path>) so URLs are bookmarkable.
//   10. Lightweight client-side search (FlexSearch) over manifest+content.
//
// Vendored libraries (loaded from index.html, present at script start):
//   - marked (window.marked)
//   - highlight.js (window.hljs)
//   - FlexSearch (window.FlexSearch) — loaded lazily on first search

(function () {
'use strict';

// ============================================
// State
// ============================================

let manifest = null;          // parsed manifest.json
let manifestPromise = null;   // in-flight fetch
let currentDocPath = null;    // currently open doc, e.g. "docs/architecture/10-motor-control.md"
let currentDocDir = '';       // directory portion of currentDocPath, used for relative link resolution
let scrollSpyObserver = null; // IntersectionObserver for TOC active state
let searchIndex = null;       // FlexSearch.Document instance, lazy-built
let searchDocs = null;        // map of id → {title, path, summary, content} for result rendering
let viewInitialized = false;  // first-time setup flag
let lastViewedPath = null;    // restored from localStorage on init

// Edit base URL — populated from manifest response (server config).
let editBaseUrl = '';

// ============================================
// Markdown rendering setup
// ============================================

function configureMarked() {
    if (!window.marked) return;
    const renderer = new marked.Renderer();

    // marked v12.0.2 calls renderer methods with POSITIONAL arguments,
    // not a token object — despite the upstream docs implying otherwise.
    // The signatures are:
    //   link(href, title, text)   // text is already-rendered HTML
    //   code(text, lang, escaped)
    // Stick to positional. Object destructuring silently produces
    // "undefined" everywhere because you'd be destructuring a string.

    // Override link renderer:
    //   - http(s)/mailto → open in new tab
    //   - anchor → leave as-is
    //   - relative .md → route through docs viewer (data-doc-link)
    //   - other relative → treat as a source citation (data-source-link)
    renderer.link = function (href, title, text) {
        if (!href) return text || '';
        const titleAttr = title ? ` title="${escapeAttr(title)}"` : '';

        if (/^https?:/i.test(href) || /^mailto:/i.test(href)) {
            return `<a href="${escapeAttr(href)}"${titleAttr} target="_blank" rel="noopener">${text}</a>`;
        }

        // Anchor within current doc
        if (href.startsWith('#')) {
            return `<a href="${escapeAttr(href)}"${titleAttr}>${text}</a>`;
        }

        // Relative path — resolve against currentDocDir
        const [pathPart, anchorPart] = href.split('#');
        const resolved = resolvePath(currentDocDir, pathPart);
        const anchor = anchorPart ? '#' + anchorPart : '';

        if (resolved.endsWith('.md')) {
            return `<a href="#docs/${escapeAttr(resolved)}${anchor}" data-doc-link="${escapeAttr(resolved)}"${titleAttr}>${text}</a>`;
        }

        // Source code path → citation link
        return `<a href="#" data-source-link="${escapeAttr(resolved)}" class="docs-source-citation"${titleAttr}>${text}</a>`;
    };

    // Highlight code blocks via highlight.js
    renderer.code = function (text, lang) {
        const safeText = text === null || text === undefined ? '' : String(text);
        if (lang && window.hljs && window.hljs.getLanguage(lang)) {
            try {
                const highlighted = window.hljs.highlight(safeText, { language: lang, ignoreIllegals: true }).value;
                return `<pre><code class="hljs language-${escapeAttr(lang)}">${highlighted}</code></pre>`;
            } catch { /* fall through to plain */ }
        }
        return `<pre><code class="hljs">${escapeHtml(safeText)}</code></pre>`;
    };

    // Image renderer: rewrite relative paths to go through /api/docs/image
    // so the docs viewer can serve images from any whitelisted root.
    // Absolute http(s) URLs are passed through unchanged.
    renderer.image = function (href, title, text) {
        if (!href) return text || '';
        const titleAttr = title ? ` title="${escapeAttr(title)}"` : '';
        const altAttr = text ? ` alt="${escapeAttr(text)}"` : '';

        let src;
        if (/^https?:/i.test(href) || href.startsWith('data:')) {
            src = href;
        } else if (href.startsWith('/api/docs/')) {
            // Already API-routed
            src = href;
        } else {
            // Resolve relative to currentDocDir against the repo root
            const resolved = resolvePath(currentDocDir, href);
            src = '/api/docs/image?path=' + encodeURIComponent(resolved);
        }

        const figureCaption = text ? `<figcaption>${escapeHtml(text)}</figcaption>` : '';
        return `<figure class="docs-figure"><img src="${escapeAttr(src)}"${altAttr}${titleAttr} loading="lazy">${figureCaption}</figure>`;
    };

    marked.use({ renderer, gfm: true, breaks: false });
}

function escapeHtml(s) {
    return String(s)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#39;');
}

function escapeAttr(s) {
    return escapeHtml(s);
}

// Resolve a relative href against a directory path. Both are repo-relative.
function resolvePath(dir, href) {
    if (href.startsWith('/')) {
        return href.replace(/^\/+/, '');
    }
    const parts = (dir ? dir.split('/').filter(Boolean) : []);
    const hrefParts = href.split('/');
    for (const p of hrefParts) {
        if (p === '..') parts.pop();
        else if (p === '.' || p === '') continue;
        else parts.push(p);
    }
    return parts.join('/');
}

// ============================================
// Math (LaTeX) pre/post-processing with KaTeX
// ============================================
//
// marked has no math support and actively damages LaTeX inside the
// markdown text:
//   - "\\" (LaTeX line break) becomes "\" (markdown escape).
//   - "$$ ... $$" gets wrapped in <p>, breaking display rendering.
//
// The fix: extract math BEFORE marked.parse() and replace each block
// with a private-use unicode placeholder. After marked produces HTML,
// swap each placeholder for KaTeX-rendered math.
//
// Placeholders use BMP private-use characters (U+E000–F8FF) so no
// markdown parser will look for or transform them.
//   Display math:  \uE000 <index> \uE001
//   Inline math:   \uE002 <index> \uE003

const MATH_DISPLAY_OPEN = '\uE000';
const MATH_DISPLAY_CLOSE = '\uE001';
const MATH_INLINE_OPEN = '\uE002';
const MATH_INLINE_CLOSE = '\uE003';

function extractMath(text) {
    const blocks = [];

    // Display math: $$ ... $$ (greedy across newlines, non-nested)
    text = text.replace(/\$\$([\s\S]+?)\$\$/g, (_, code) => {
        const idx = blocks.length;
        blocks.push({ display: true, code });
        return MATH_DISPLAY_OPEN + idx + MATH_DISPLAY_CLOSE;
    });

    // Inline math: $...$ — must not have whitespace adjacent to the
    // delimiters, must not span newlines, and must not be a $ amount
    // ("$5" or "$10 and $20"). The first character after $ and the
    // last before the closing $ must be non-space, non-digit when at
    // string boundaries.
    text = text.replace(/(^|[^\\$\d])\$([^\s$][^\n$]*?[^\s$]|\S)\$(?!\d)/g, (match, prefix, code) => {
        const idx = blocks.length;
        blocks.push({ display: false, code });
        return prefix + MATH_INLINE_OPEN + idx + MATH_INLINE_CLOSE;
    });

    return { text, blocks };
}

function reinsertMath(html, blocks) {
    if (!blocks || blocks.length === 0) return html;
    if (!window.katex) {
        // KaTeX not loaded; show raw LaTeX in a <code> as a graceful fallback.
        return html
            .replace(new RegExp(MATH_DISPLAY_OPEN + '(\\d+)' + MATH_DISPLAY_CLOSE, 'g'),
                (_, i) => `<pre><code>${escapeHtml(blocks[Number(i)].code)}</code></pre>`)
            .replace(new RegExp(MATH_INLINE_OPEN + '(\\d+)' + MATH_INLINE_CLOSE, 'g'),
                (_, i) => `<code>${escapeHtml(blocks[Number(i)].code)}</code>`);
    }

    return html
        .replace(new RegExp(MATH_DISPLAY_OPEN + '(\\d+)' + MATH_DISPLAY_CLOSE, 'g'), (_, i) => {
            const block = blocks[Number(i)];
            try {
                return window.katex.renderToString(block.code, {
                    displayMode: true,
                    throwOnError: false,
                    output: 'html',
                });
            } catch (err) {
                return `<pre class="docs-math-error" title="${escapeAttr(err.message)}">${escapeHtml(block.code)}</pre>`;
            }
        })
        .replace(new RegExp(MATH_INLINE_OPEN + '(\\d+)' + MATH_INLINE_CLOSE, 'g'), (_, i) => {
            const block = blocks[Number(i)];
            try {
                return window.katex.renderToString(block.code, {
                    displayMode: false,
                    throwOnError: false,
                    output: 'html',
                });
            } catch (err) {
                return `<code class="docs-math-error" title="${escapeAttr(err.message)}">${escapeHtml(block.code)}</code>`;
            }
        });
}

// ============================================
// Frontmatter parsing
// ============================================
//
// We accept YAML-style frontmatter at the very top of the doc, between
// `---` lines. Only a tiny subset is supported: scalar key: value and
// list-of-strings (each on its own indented `- value` line).

function parseFrontmatter(text) {
    if (!text.startsWith('---\n') && !text.startsWith('---\r\n')) {
        return { frontmatter: null, body: text };
    }
    const end = text.indexOf('\n---', 4);
    if (end === -1) {
        return { frontmatter: null, body: text };
    }
    const block = text.substring(4, end);
    const body = text.substring(end + 4).replace(/^\r?\n/, '');

    const fm = {};
    const lines = block.split(/\r?\n/);
    let currentKey = null;
    for (const line of lines) {
        if (!line.trim()) continue;
        if (/^\s+-\s+/.test(line)) {
            // list item under previous key
            if (currentKey && Array.isArray(fm[currentKey])) {
                fm[currentKey].push(line.replace(/^\s+-\s+/, '').trim());
            }
            continue;
        }
        const match = line.match(/^([a-zA-Z0-9_-]+)\s*:\s*(.*)$/);
        if (!match) continue;
        const key = match[1];
        const value = match[2];
        if (value === '') {
            fm[key] = [];
            currentKey = key;
        } else {
            fm[key] = value;
            currentKey = null;
        }
    }
    return { frontmatter: fm, body };
}

function renderFrontmatter(fm) {
    if (!fm) return '';
    const parts = [];
    if (fm['last-verified']) {
        parts.push(`<span><span class="docs-frontmatter-key">Last verified:</span> <span class="docs-frontmatter-val">${escapeHtml(fm['last-verified'])}</span></span>`);
    }
    if (Array.isArray(fm.sources) && fm.sources.length > 0) {
        const list = fm.sources.map((s) => `<a href="#" data-source-link="${escapeAttr(s)}" class="docs-source-citation">${escapeHtml(s)}</a>`).join(', ');
        parts.push(`<span><span class="docs-frontmatter-key">Sources:</span> <span class="docs-frontmatter-val">${list}</span></span>`);
    }
    if (parts.length === 0) return '';
    return `<div class="docs-frontmatter">${parts.join('')}</div>`;
}

// ============================================
// Source citation linkification
// ============================================
//
// After marked has rendered, walk text nodes inside the content pane
// and replace bare "path/to/file.cpp:NN-MM" patterns with clickable
// links. We don't do this in the marked renderer because the patterns
// appear inside paragraph and list-item text, not inside markdown link
// syntax.

const CITATION_RE = /\b((?:[\w.-]+\/)+[\w.-]+\.(?:cpp|h|hpp|c|cc|js|mjs|ts|json|ini|cfg|toml|yaml|yml|py|html|css))(?::(\d+)(?:-(\d+))?)?\b/g;

function linkifyCitations(root) {
    const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT, {
        acceptNode(node) {
            // Skip text inside <a>, <code>, <pre>, and <script>
            const parent = node.parentNode;
            if (!parent) return NodeFilter.FILTER_REJECT;
            const tag = parent.nodeName;
            if (tag === 'A' || tag === 'CODE' || tag === 'PRE' || tag === 'SCRIPT') {
                return NodeFilter.FILTER_REJECT;
            }
            return CITATION_RE.test(node.nodeValue) ? NodeFilter.FILTER_ACCEPT : NodeFilter.FILTER_REJECT;
        },
    });

    const targets = [];
    let n;
    while ((n = walker.nextNode())) targets.push(n);

    for (const node of targets) {
        const text = node.nodeValue;
        CITATION_RE.lastIndex = 0;
        let match;
        const frag = document.createDocumentFragment();
        let lastIdx = 0;
        while ((match = CITATION_RE.exec(text)) !== null) {
            if (match.index > lastIdx) {
                frag.appendChild(document.createTextNode(text.slice(lastIdx, match.index)));
            }
            const [, file, start, end] = match;
            const a = document.createElement('a');
            a.href = '#';
            a.className = 'docs-source-citation';
            a.dataset.sourceLink = file;
            if (start) a.dataset.sourceStart = start;
            if (end) a.dataset.sourceEnd = end;
            a.textContent = match[0];
            frag.appendChild(a);
            lastIdx = match.index + match[0].length;
        }
        if (lastIdx < text.length) {
            frag.appendChild(document.createTextNode(text.slice(lastIdx)));
        }
        node.parentNode.replaceChild(frag, node);
    }
}

// ============================================
// Manifest fetching & sidebar tree
// ============================================

async function fetchManifest() {
    if (manifest) return manifest;
    if (manifestPromise) return manifestPromise;
    manifestPromise = fetch('/api/docs/manifest')
        .then((r) => {
            if (!r.ok) throw new Error('failed to load manifest');
            return r.json();
        })
        .then((m) => {
            manifest = m;
            if (typeof m.editBaseUrl === 'string') editBaseUrl = m.editBaseUrl;
            return m;
        })
        .catch((err) => {
            manifestPromise = null;
            throw err;
        });
    return manifestPromise;
}

function renderSidebarTree() {
    const tree = document.getElementById('docs-tree');
    if (!tree || !manifest) return;
    tree.innerHTML = '';

    for (const cat of manifest.categories) {
        const catEl = document.createElement('div');
        catEl.className = 'docs-category';
        if (cat.collapsed) catEl.classList.add('collapsed');

        const title = document.createElement('div');
        title.className = 'docs-category-title';
        title.textContent = cat.title;
        title.addEventListener('click', () => catEl.classList.toggle('collapsed'));
        catEl.appendChild(title);

        const list = document.createElement('ul');
        list.className = 'docs-category-list';
        for (const doc of cat.docs) {
            const li = document.createElement('li');
            const link = document.createElement('div');
            link.className = 'docs-doc-link';
            link.textContent = doc.title;
            link.dataset.path = doc.path;
            link.addEventListener('click', () => openDoc(doc.path));
            li.appendChild(link);
            list.appendChild(li);
        }
        catEl.appendChild(list);
        tree.appendChild(catEl);
    }
    updateActiveSidebarItem();
}

function updateActiveSidebarItem() {
    document.querySelectorAll('.docs-doc-link').forEach((el) => {
        el.classList.toggle('active', el.dataset.path === currentDocPath);
    });
}

// ============================================
// Doc fetching & rendering
// ============================================

async function openDoc(path) {
    currentDocPath = path;
    currentDocDir = path.substring(0, path.lastIndexOf('/'));
    try { localStorage.setItem('omni-docs-last', path); } catch { /* ignore */ }
    updateActiveSidebarItem();

    if (location.hash !== '#docs/' + path) {
        // Update hash without triggering hashchange handler
        suppressNextHashChange = true;
        location.hash = 'docs/' + path;
    }

    const content = document.getElementById('docs-content');
    content.innerHTML = '<div class="docs-loading">Loading…</div>';

    try {
        const res = await fetch('/api/docs/file?path=' + encodeURIComponent(path));
        if (!res.ok) {
            const msg = await res.text();
            throw new Error(`${res.status}: ${msg}`);
        }
        const text = await res.text();
        renderDoc(text, path);
    } catch (err) {
        content.innerHTML = `<div class="docs-error">Failed to load <code>${escapeHtml(path)}</code>: ${escapeHtml(err.message)}</div>`;
        clearToc();
    }
}

function renderDoc(text, path) {
    const { frontmatter, body } = parseFrontmatter(text);

    // Extract math blocks BEFORE marked sees them — marked would
    // otherwise mangle "\\" line breaks and wrap display math in <p>.
    const { text: bodyNoMath, blocks: mathBlocks } = extractMath(body);

    let html = window.marked.parse(bodyNoMath);
    html = reinsertMath(html, mathBlocks);

    const content = document.getElementById('docs-content');
    content.innerHTML = `
        <div class="docs-doc-header">
            <div></div>
            ${editBaseUrl ? `<a class="docs-edit-link" href="${escapeAttr(editBaseUrl + '/' + path)}" target="_blank" rel="noopener">View source</a>` : ''}
        </div>
        ${renderFrontmatter(frontmatter)}
        <div class="docs-md">${html}</div>
    `;

    // Post-render passes
    const md = content.querySelector('.docs-md');
    linkifyCitations(md);
    buildToc();
    content.scrollTop = 0;
}

function renderLanding() {
    const content = document.getElementById('docs-content');
    if (!content || !manifest) return;
    currentDocPath = null;
    currentDocDir = '';
    updateActiveSidebarItem();
    clearToc();

    const cats = manifest.categories.map((cat) => {
        const docs = cat.docs.map((doc) => `
            <div class="docs-landing-doc" data-path="${escapeAttr(doc.path)}">
                <div class="docs-landing-doc-title">${escapeHtml(doc.title)}</div>
                ${doc.summary ? `<div class="docs-landing-doc-summary">${escapeHtml(doc.summary)}</div>` : ''}
            </div>
        `).join('');
        return `
            <div class="docs-landing-cat">
                <h2>${escapeHtml(cat.title)}</h2>
                ${cat.description ? `<div class="docs-landing-cat-desc">${escapeHtml(cat.description)}</div>` : ''}
                ${docs}
            </div>
        `;
    }).join('');

    content.innerHTML = `
        <div class="docs-landing">
            <h1>Documentation</h1>
            <p>Pick a topic from the categories below or use the sidebar.</p>
            ${cats}
        </div>
    `;

    content.querySelectorAll('.docs-landing-doc').forEach((el) => {
        el.addEventListener('click', () => openDoc(el.dataset.path));
    });
}

// ============================================
// TOC + scroll-spy
// ============================================

function clearToc() {
    const list = document.getElementById('docs-toc-list');
    if (list) list.innerHTML = '';
    if (scrollSpyObserver) {
        scrollSpyObserver.disconnect();
        scrollSpyObserver = null;
    }
}

function buildToc() {
    clearToc();
    const md = document.querySelector('#docs-content .docs-md');
    const list = document.getElementById('docs-toc-list');
    if (!md || !list) return;

    const headings = md.querySelectorAll('h2, h3');
    if (headings.length === 0) return;

    const items = [];
    headings.forEach((h, i) => {
        // Generate slug id
        const slug = (h.textContent || '').toLowerCase()
            .replace(/[^a-z0-9\s-]/g, '')
            .trim()
            .replace(/\s+/g, '-')
            .substring(0, 60) || `section-${i}`;
        h.id = slug;

        const li = document.createElement('li');
        li.className = h.tagName.toLowerCase();
        const a = document.createElement('a');
        a.href = `#docs/${currentDocPath || ''}#${slug}`;
        a.textContent = h.textContent || '';
        a.addEventListener('click', (e) => {
            e.preventDefault();
            h.scrollIntoView({ behavior: 'smooth', block: 'start' });
        });
        li.appendChild(a);
        list.appendChild(li);
        items.push({ heading: h, li });
    });

    // Scroll-spy via IntersectionObserver
    if ('IntersectionObserver' in window) {
        scrollSpyObserver = new IntersectionObserver((entries) => {
            for (const entry of entries) {
                if (entry.isIntersecting) {
                    items.forEach((it) => it.li.classList.toggle('active', it.heading === entry.target));
                }
            }
        }, {
            root: document.getElementById('docs-content'),
            rootMargin: '-20% 0px -70% 0px',
            threshold: 0,
        });
        items.forEach((it) => scrollSpyObserver.observe(it.heading));
    }
}

// ============================================
// Source citation modal
// ============================================

async function openSourceModal(path, start, end) {
    const modal = document.getElementById('docs-source-modal');
    const code = document.getElementById('docs-source-code');
    const title = document.getElementById('docs-source-title');
    if (!modal || !code || !title) return;

    title.textContent = path + (start ? `:${start}${end ? '-' + end : ''}` : '');
    code.textContent = 'Loading…';
    modal.classList.add('visible');

    try {
        const params = new URLSearchParams({ path });
        if (start) params.set('start', String(start));
        if (end) params.set('end', String(end));
        const res = await fetch('/api/docs/source?' + params.toString());
        if (!res.ok) {
            const msg = await res.text();
            throw new Error(`${res.status}: ${msg}`);
        }
        const data = await res.json();

        // Try to highlight by file extension
        const ext = path.split('.').pop();
        const langMap = { cpp: 'cpp', h: 'cpp', hpp: 'cpp', c: 'c', cc: 'cpp', js: 'javascript', mjs: 'javascript', ts: 'typescript', json: 'json', py: 'python', html: 'xml', css: 'css' };
        const lang = langMap[ext];

        let html = '';
        const lines = data.content.split('\n');
        const startLine = data.start;
        const lineNumWidth = String(data.start + lines.length - 1).length;

        if (lang && window.hljs && window.hljs.getLanguage(lang)) {
            const highlighted = window.hljs.highlight(data.content, { language: lang, ignoreIllegals: true }).value;
            const hLines = highlighted.split('\n');
            html = hLines.map((l, i) => {
                const num = String(startLine + i).padStart(lineNumWidth, ' ');
                return `<span style="opacity:0.4;user-select:none;">${num}</span>  ${l}`;
            }).join('\n');
        } else {
            html = lines.map((l, i) => {
                const num = String(startLine + i).padStart(lineNumWidth, ' ');
                return `<span style="opacity:0.4;user-select:none;">${num}</span>  ${escapeHtml(l)}`;
            }).join('\n');
        }
        code.innerHTML = html;
    } catch (err) {
        code.textContent = 'Failed to load source: ' + err.message;
    }
}

function closeSourceModal() {
    document.getElementById('docs-source-modal')?.classList.remove('visible');
}

// ============================================
// Search (FlexSearch)
// ============================================

async function buildSearchIndex() {
    if (searchIndex) return;
    if (!window.FlexSearch) {
        // Lazy-load
        await loadScript('vendor/flexsearch.bundle.min.js');
    }
    if (!window.FlexSearch || !manifest) return;

    searchDocs = [];
    let id = 0;
    for (const cat of manifest.categories) {
        for (const doc of cat.docs) {
            searchDocs.push({ id: id++, title: doc.title, path: doc.path, summary: doc.summary || '', category: cat.title, content: '' });
        }
    }

    // Fetch each doc once for content. This is the slowest part — done
    // lazily on first search interaction so it doesn't block view open.
    const fetches = searchDocs.map((d) =>
        fetch('/api/docs/file?path=' + encodeURIComponent(d.path))
            .then((r) => r.ok ? r.text() : '')
            .then((t) => { d.content = stripMarkdown(t).slice(0, 8000); })
            .catch(() => {}),
    );
    await Promise.all(fetches);

    searchIndex = new window.FlexSearch.Document({
        document: {
            id: 'id',
            index: ['title', 'summary', 'content'],
            store: ['title', 'path', 'summary', 'category'],
        },
        tokenize: 'forward',
        cache: true,
    });
    for (const d of searchDocs) searchIndex.add(d);
}

function stripMarkdown(text) {
    // Quick & dirty: remove code fences, frontmatter, and most markdown syntax.
    return text
        .replace(/^---[\s\S]*?\n---\n/, '')
        .replace(/```[\s\S]*?```/g, ' ')
        .replace(/`[^`]*`/g, ' ')
        .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1')
        .replace(/[#*_>~|-]/g, ' ')
        .replace(/\s+/g, ' ');
}

function loadScript(src) {
    return new Promise((resolve, reject) => {
        const s = document.createElement('script');
        s.src = src;
        s.onload = resolve;
        s.onerror = reject;
        document.head.appendChild(s);
    });
}

let searchTimer = null;
async function handleSearchInput(value) {
    clearTimeout(searchTimer);
    const resultsEl = document.getElementById('docs-search-results');
    if (!value || value.trim().length < 2) {
        if (resultsEl) { resultsEl.hidden = true; resultsEl.innerHTML = ''; }
        return;
    }
    searchTimer = setTimeout(async () => {
        if (!searchIndex) {
            resultsEl.hidden = false;
            resultsEl.innerHTML = '<div class="docs-search-empty">Building index…</div>';
            await buildSearchIndex();
        }
        if (!searchIndex) {
            resultsEl.innerHTML = '<div class="docs-search-empty">Search unavailable.</div>';
            return;
        }
        const hits = searchIndex.search(value, { limit: 12, enrich: true });
        const seen = new Set();
        const flat = [];
        for (const field of hits) {
            for (const item of field.result) {
                if (seen.has(item.id)) continue;
                seen.add(item.id);
                flat.push(item.doc);
            }
        }
        if (flat.length === 0) {
            resultsEl.innerHTML = '<div class="docs-search-empty">No results.</div>';
            resultsEl.hidden = false;
            return;
        }
        resultsEl.innerHTML = flat.slice(0, 10).map((d) => `
            <div class="docs-search-result" data-path="${escapeAttr(d.path)}">
                <div class="docs-search-result-title">${escapeHtml(d.title)}</div>
                <div class="docs-search-result-snippet">${escapeHtml(d.category)}${d.summary ? ' — ' + escapeHtml(d.summary) : ''}</div>
            </div>
        `).join('');
        resultsEl.hidden = false;
        resultsEl.querySelectorAll('.docs-search-result').forEach((el) => {
            el.addEventListener('click', () => {
                openDoc(el.dataset.path);
                resultsEl.hidden = true;
                document.getElementById('docs-search').value = '';
            });
        });
    }, 200);
}

// ============================================
// Hash routing
// ============================================
//
// Hash format: #docs/<repo-relative-path>[#anchor]
// Examples:
//   #docs/docs/architecture/10-motor-control.md
//   #docs/docs/architecture/10-motor-control.md#tldr

let suppressNextHashChange = false;

function handleHashChange() {
    if (suppressNextHashChange) {
        suppressNextHashChange = false;
        return;
    }
    const hash = location.hash;
    if (!hash.startsWith('#docs/')) return;
    const rest = hash.substring('#docs/'.length);
    const [path, anchor] = rest.split('#');
    if (!path) {
        renderLanding();
        return;
    }
    if (path !== currentDocPath) {
        openDoc(path).then(() => {
            if (anchor) {
                const el = document.getElementById(anchor);
                if (el) el.scrollIntoView({ behavior: 'smooth' });
            }
        });
    } else if (anchor) {
        const el = document.getElementById(anchor);
        if (el) el.scrollIntoView({ behavior: 'smooth' });
    }
}

// ============================================
// View activation
// ============================================

async function activateDocsView() {
    if (!viewInitialized) {
        viewInitialized = true;
        configureMarked();

        // Enable the highlight.js stylesheet (it's loaded with disabled=true
        // to avoid affecting other views).
        const hljsTheme = document.getElementById('hljs-theme');
        if (hljsTheme) hljsTheme.disabled = false;

        // Theme-aware highlight.js stylesheet swap
        const swapHljsTheme = () => {
            const link = document.getElementById('hljs-theme');
            if (!link) return;
            const dark = !window.Theme || window.Theme.current() === 'dark';
            link.href = dark ? 'vendor/highlight-dark.css' : 'vendor/highlight-light.css';
        };
        swapHljsTheme();
        if (window.Theme && typeof window.Theme.onChange === 'function') {
            window.Theme.onChange(swapHljsTheme);
        }

        // Wire delegated click handlers on the content pane
        const content = document.getElementById('docs-content');
        content.addEventListener('click', (e) => {
            const docLink = e.target.closest('[data-doc-link]');
            if (docLink) {
                e.preventDefault();
                openDoc(docLink.dataset.docLink);
                return;
            }
            const sourceLink = e.target.closest('[data-source-link]');
            if (sourceLink) {
                e.preventDefault();
                const start = sourceLink.dataset.sourceStart ? Number(sourceLink.dataset.sourceStart) : undefined;
                const end = sourceLink.dataset.sourceEnd ? Number(sourceLink.dataset.sourceEnd) : undefined;
                openSourceModal(sourceLink.dataset.sourceLink, start, end);
                return;
            }
        });

        // Modal close
        document.getElementById('docs-source-close')?.addEventListener('click', closeSourceModal);
        document.getElementById('docs-source-modal')?.addEventListener('click', (e) => {
            if (e.target.id === 'docs-source-modal') closeSourceModal();
        });

        // Search input
        const searchInput = document.getElementById('docs-search');
        if (searchInput) {
            searchInput.addEventListener('input', (e) => handleSearchInput(e.target.value));
            searchInput.addEventListener('focus', () => {
                if (!searchIndex) buildSearchIndex();
            });
        }
        document.addEventListener('click', (e) => {
            if (!e.target.closest('.docs-search-wrap')) {
                const r = document.getElementById('docs-search-results');
                if (r) r.hidden = true;
            }
        });

        // Restore last viewed doc
        try { lastViewedPath = localStorage.getItem('omni-docs-last'); } catch { /* ignore */ }
    }

    // Fetch manifest if needed
    if (!manifest) {
        try {
            await fetchManifest();
            renderSidebarTree();
        } catch (err) {
            document.getElementById('docs-tree').innerHTML = `<div class="docs-error">Failed to load manifest: ${escapeHtml(err.message)}</div>`;
            return;
        }
    }

    // If hash points at a doc, open it; otherwise restore last viewed or show landing
    const hash = location.hash;
    if (hash.startsWith('#docs/')) {
        handleHashChange();
    } else if (lastViewedPath) {
        openDoc(lastViewedPath);
    } else {
        renderLanding();
    }
}

// ============================================
// Init
// ============================================

function init() {
    // Side-nav item handler is already wired by app.js — viewChanged event
    // is the right hook for activating this view.
    if (window.App && typeof window.App.on === 'function') {
        window.App.on('viewChanged', (view) => {
            if (view === 'docs') activateDocsView();
        });
    }

    // Top-bar Docs button
    const btn = document.getElementById('docs-btn');
    if (btn && window.App && typeof window.App.switchView === 'function') {
        btn.addEventListener('click', () => window.App.switchView('docs'));
    }

    // Hash routing
    window.addEventListener('hashchange', handleHashChange);
}

document.addEventListener('DOMContentLoaded', init);
})();
