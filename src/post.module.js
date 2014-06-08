
// Node.js
if (typeof module !== 'undefined' && module.exports) {
    module.exports = Module;
}
// AMD / RequireJS
else if (typeof define !== 'undefined' && define.amd) {
    define([], function () {
        return Module;
    });
}
// included directly via <script> tag
else {
    root.recast = Module;
}

