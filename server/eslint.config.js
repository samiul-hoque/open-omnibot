import js from '@eslint/js';
import globals from 'globals';

export default [
    {
        ignores: ['src/web/public/vendor/**', 'src/config.local.js'],
    },
    js.configs.recommended,
    {
        languageOptions: {
            ecmaVersion: 2022,
            sourceType: 'module',
            globals: {
                ...globals.node,
            },
        },
        rules: {
            'no-unused-vars': ['error', { argsIgnorePattern: '^_', varsIgnorePattern: '^_' }],
            'no-console': 'off',
            'semi': ['error', 'always'],
            'quotes': ['error', 'single', { avoidEscape: true }],
            'indent': ['error', 4],
            'comma-dangle': ['error', 'always-multiline'],
            'eqeqeq': ['error', 'always'],
            'no-var': 'error',
            'prefer-const': 'error',
        },
    },
    // Browser files (web UI) — ES modules
    {
        files: ['src/web/public/**/*.js'],
        languageOptions: {
            ecmaVersion: 2022,
            sourceType: 'module',
            globals: {
                ...globals.browser,
            },
        },
    },
    // Classic scripts (loaded without type="module"): still parsed as scripts
    {
        files: ['src/web/public/theme.js', 'src/web/public/docs.js'],
        languageOptions: {
            sourceType: 'script',
        },
    },
    // docs.js wraps its entire body in an IIFE at column 0 and uses
    // vendored globals (marked, hljs, katex, FlexSearch) loaded from
    // index.html. Skip the strict indent + no-undef rules for it.
    {
        files: ['src/web/public/docs.js'],
        languageOptions: {
            globals: {
                marked: 'readonly',
                hljs: 'readonly',
                katex: 'readonly',
                FlexSearch: 'readonly',
            },
        },
        rules: {
            'indent': 'off',
        },
    },
];
