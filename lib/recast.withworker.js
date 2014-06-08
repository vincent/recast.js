/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global setImmediate: false, setTimeout: false, console: false, module: true, process: true, define: true */
(function () {
    'use strict';

    // Node.js
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = recast;
    }
    // AMD / RequireJS
    else if (typeof define !== 'undefined' && define.amd) {
        define([], function () {
            return recast;
        });
    }
    // included directly via <script> tag
    else {
        root.recast = recast;
    }

})();