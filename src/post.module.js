// With MODULARIZE=1, Emscripten handles all CommonJS/AMD/ESM exports.

// Signal readiness when running as a Worker
if (typeof importScripts === 'function') {
  postMessage({ type: 'ready' });
}
