import js from '@eslint/js';
import globals from 'globals';

const styleRules = {
  'quotes': ['error', 'single'],
  'semi': ['error', 'always'],
  'indent': ['error', 2, { SwitchCase: 1 }],
  'no-unused-vars': 'warn',
};

export default [
  {
    ignores: ['lib/', 'recastnavigation/', 'node_modules/'],
  },

  // bin/ — CommonJS, Node.js globals
  {
    files: ['bin/**/*.js'],
    ...js.configs.recommended,
    languageOptions: {
      ecmaVersion: 2022,
      sourceType: 'commonjs',
      globals: globals.node,
    },
    rules: {
      ...js.configs.recommended.rules,
      ...styleRules,
    },
  },

  // tests/ — ES modules, Node.js globals (URL, Buffer available in Node 18+)
  {
    files: ['tests/unit/**/*.js'],
    ...js.configs.recommended,
    languageOptions: {
      ecmaVersion: 2022,
      sourceType: 'module',
      globals: globals.node,
    },
    rules: {
      ...js.configs.recommended.rules,
      ...styleRules,
    },
  },

  // src/ — Emscripten-injected scripts; apply only recommended rules
  // Inline /*global*/ comments in each file declare file-specific Emscripten globals
  {
    files: ['src/**/*.js'],
    ...js.configs.recommended,
    languageOptions: {
      ecmaVersion: 2022,
      sourceType: 'script',
      globals: {
        ...globals.browser,
        ...globals.node,
        // Emscripten runtime globals not covered by browser/node sets
        Module: 'writable',
        FS: 'readonly',
        HEAPF32: 'readonly',
        agentPool: 'writable',
        agentPoolBuffer: 'writable',
        mergeInto: 'readonly',
        LibraryManager: 'readonly',
      },
    },
    rules: {
      ...js.configs.recommended.rules,
      'no-unused-vars': 'warn',
      // Emscripten code intentionally overrides browser globals (e.g. postMessage)
      // and inline /*global*/ comments re-declare config globals
      'no-global-assign': 'off',
      'no-redeclare': 'off',
    },
  },
];
