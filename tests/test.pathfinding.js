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

    test.expect(5);

    settings(recast);

    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

        recast.buildTiled();

        /**
         * Find a random navigable point on this mesh
         */
        recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){
            test.ok(typeof pt1x === 'number' && typeof pt1y === 'number' && typeof pt1z === 'number', 'find a random point');

            var extend = 3;

            /**
             * Find the nearest navigable point from 0,0,0 with a maximum extend
             */
            recast.findNearestPoint(0, 0, 0, extend, extend, extend, recast.cb(function(pt2x, pt2y, pt2z){
                test.ok(typeof pt2x === 'number' && typeof pt2y === 'number' && typeof pt2z === 'number', 'find the nearest point');

                /**
                 * Find the nearest navigable polygon from 0,0,0 with a maximum extend
                 */
                recast.findNearestPoly(0, 0, 0, extend, extend, extend, recast.cb(function(polygon){
                    test.ok(polygon.vertices, 'origin poly has some vertices');

                    test.ok(polygon.vertices && typeof polygon.vertices.length !== 'undefined', 'origin poly has ' + polygon.vertices.length + ' polygon.vertices');

                    /**
                     * Find the shortest possible path from pt1 to pt2
                     */
                    recast.findPath(pt1x, pt1y, pt1z, pt2x, pt2y, pt2z, 1000, recast.cb(function(path){
                        test.ok(path && typeof path.length !== 'undefined', 'found path has ' + path.length + ' segments');

                        test.done();
                    }));
                }));
            }));

        }));
    });
};


