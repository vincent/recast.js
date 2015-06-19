/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true, Buffer: true */
'use strict';

var fs       = require('fs');
var recast   = require('../lib/recast');
var settings = require('./settings');

exports['save a tiled navmesh'] = function(test) {

    settings(recast);

        console.log('load');

    recast.OBJLoader('nav_test.obj', function(){

        console.log('build');

        recast.buildTiled();

        console.log('save');

        recast.saveTileMesh('./navmesh.bin', recast.cb(function (error, serialized) {

            if (fs.writeFile) {

                var buffer = new Buffer(serialized.length);

                for (var i = 0; i < serialized.length; i++) {
                    buffer.writeUInt8(serialized[i], i);
                }

                fs.writeFile('./navmesh.bin', buffer, function (err) {
                    if (err) throw err;
                    test.done();
                });

            } else test.done();
        }));
    });
};

