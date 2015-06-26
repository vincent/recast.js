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

exports['load a tiled navmesh'] = function(test) {

    test.expect(2);

    settings(recast);

    // recast.OBJLoader('nav_test.obj', function(){

        recast.loadTileMesh('./navmesh.dist.bin', recast.cb(function(){

            /**
             * Find a random navigable point on this mesh
             */
            recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){

                test.ok(typeof pt1x === 'number' && typeof pt1y === 'number' && typeof pt1z === 'number', 'find a random point');

                /**
                 * Find the nearest navigable point from pt1x,pt1y,pt1z with a maximum extend of 10,10,10
                 */
                recast.findNearestPoint(pt1x, pt1y, pt1z, 10, 10, 10, recast.cb(function(pt2x, pt2y, pt2z){

                    test.ok(typeof pt2x === 'number' && typeof pt2y === 'number' && typeof pt2z === 'number', 'find the nearest point');

                    test.done();
                }));
            }));
        }));
    // });
};

