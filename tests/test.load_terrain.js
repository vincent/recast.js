/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true */
'use strict';

var recast   = require('../lib/recast');
var settings = require('./settings');

exports['load an .obj file'] = function(test) {

    settings(recast);

    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

        recast.buildTiled();

        test.done();
    });
};


