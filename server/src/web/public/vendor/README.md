# Vendored libraries

Third-party JavaScript and CSS bundled with the dashboard. We vendor (rather than CDN-load) so the robot's web UI works on its own AP-mode network with no internet access.

| File | Library | Version | License | Source |
|---|---|---|---|---|
| `marked.min.js` | marked | 12.0.2 | MIT | https://github.com/markedjs/marked |
| `highlight.min.js` | highlight.js | 11.9.0 | BSD-3-Clause | https://github.com/highlightjs/highlight.js |
| `highlight-dark.css` | highlight.js Atom One Dark | 11.9.0 | BSD-3-Clause | (same) |
| `highlight-light.css` | highlight.js Atom One Light | 11.9.0 | BSD-3-Clause | (same) |
| `flexsearch.bundle.min.js` | FlexSearch | 0.7.43 (bundle) | Apache-2.0 | https://github.com/nextapps-de/flexsearch |
| `katex/katex.min.js` | KaTeX | 0.16.11 | MIT | https://github.com/KaTeX/KaTeX |
| `katex/katex.min.css` | KaTeX styles (woff2-only, woff/ttf URLs stripped) | 0.16.11 | MIT | (same) |
| `katex/fonts/*.woff2` | KaTeX font set (20 files, ~300 KB) | 0.16.11 | OFL-1.1 | (same) |

License headers are preserved at the top of each minified file.

## How to refresh

Each library was downloaded from jsDelivr at the version pinned above. To refresh, re-run:

```bash
cd server/src/web/public/vendor
curl -sSL -o marked.min.js https://cdn.jsdelivr.net/npm/marked@12.0.2/marked.min.js
curl -sSL -o highlight.min.js "https://cdn.jsdelivr.net/gh/highlightjs/cdn-release@11.9.0/build/highlight.min.js"
curl -sSL -o highlight-dark.css "https://cdn.jsdelivr.net/gh/highlightjs/cdn-release@11.9.0/build/styles/atom-one-dark.min.css"
curl -sSL -o highlight-light.css "https://cdn.jsdelivr.net/gh/highlightjs/cdn-release@11.9.0/build/styles/atom-one-light.min.css"
curl -sSL -o flexsearch.bundle.min.js "https://cdn.jsdelivr.net/npm/flexsearch@0.7.43/dist/flexsearch.bundle.min.js"

# KaTeX (math rendering) — JS, CSS (with woff/ttf stripped), and woff2 fonts.
mkdir -p katex/fonts
curl -sSL -o katex/katex.min.js  "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"
curl -sSL -o katex/katex.min.css "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css"
for f in $(grep -oE 'KaTeX_[A-Za-z0-9_-]+\.woff2' katex/katex.min.css | sort -u); do
  curl -sSL -o "katex/fonts/$f" "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/fonts/$f"
done
# Strip woff/ttf URLs from the CSS so the browser only fetches woff2.
python3 -c "
import re
with open('katex/katex.min.css','r') as f: css=f.read()
css = re.sub(r',url\(fonts/KaTeX_[A-Za-z0-9_-]+\.woff\) format\(\"woff\"\)', '', css)
css = re.sub(r',url\(fonts/KaTeX_[A-Za-z0-9_-]+\.ttf\) format\(\"truetype\"\)', '', css)
open('katex/katex.min.css','w').write(css)
"
```

When bumping versions, also bump the table above.
